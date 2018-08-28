// Qt library
#include <QFileDialog>
#include <QInputDialog>
#include <QString>
#include <QDateTime>
#include <QDebug>
#include <QMessageBox>
#include <QKeyEvent>
#include <QKeySequence>
#include <QFileInfo>
#include <QLabel>
#include <QFont>
#include <QHeaderView>
#include <helpdialog.h>
#include <QFile>
#include <QTextStream>
#include <QDir>

// VLCQt library
#include <VLCQtCore/Common.h>
#include <VLCQtCore/Instance.h>
#include <VLCQtCore/Media.h>
#include <VLCQtCore/MediaPlayer.h>
#include <VLCQtCore/Video.h>

// C++ std library
#include <exception>

// OpenCV library
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL library
#include <vtkRenderWindow.h>

// custom library
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "bmscore.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _media(0)
{
    ui->setupUi(this);

    // global settings
    dataCount = 0;      // init counter with 0
    useDefalutValue = false;
    ifConnect2DCam = ifConnect3DCam = false;
    snapshotPath = QDir::homePath() + "/bms_snapshots";
    // create snapshot path
    if (!QDir().exists(snapshotPath))
    {
        bool r = QDir().mkpath(snapshotPath);
        // save log to file
    }
    ifm3dViewer.setSnapshotPath(snapshotPath);
    vlc2DcameraSSThd.setSnapshotPath(snapshotPath);
    filesTabVlcSSThd.setSnapshotPath(snapshotPath);

    // 2d camera settings
    ui->camera->setStyleSheet("border:1px solid black");
    _instance = new VlcInstance(VlcCommon::args(), this);
    _player = new VlcMediaPlayer(_instance);
    _player->setVideoWidget(ui->camera);
    ui->camera->setMediaPlayer(_player);

    // 2d image settings
    ui->imageTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->imageTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->imageTableWidget->setColumnCount(2);

    // settings of header
    QFont font = ui->imageTableWidget->horizontalHeader()->font();
    font.setBold(true);
    ui->imageTableWidget->horizontalHeader()->setFont(font);

//    ui->imageTableWidget->setColumnWidth(0, 31);
    ui->imageTableWidget->setColumnWidth(0, 115);
    ui->imageTableWidget->horizontalHeader()->setStretchLastSection(true);
//    ui->imageTableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->imageTableWidget->horizontalHeader()->setSectionsClickable(false);
//    ui->imageTableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->imageTableWidget->horizontalHeader()->setStyleSheet("QHeaderView::section{background:lightgray;}");
    ui->imageTableWidget->setStyleSheet("selection-background-color:skyblue;");
    ui->imageTableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->imageTableWidget->setSelectionMode(QAbstractItemView::SingleSelection);

    QStringList tableHeader;

    tableHeader << tr("2D Images") << tr("3D Images");
    ui->imageTableWidget->setHorizontalHeaderLabels(tableHeader);

    imgMarkScene = new ImgMarkScene(this);

    ui->waistHgtInfoLabel->setText("m");
    ui->waistHgtInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->withersHgtInfoLabel->setText("m");
    ui->withersHgtInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->bodyLgtInfoLabel->setText("m");
    ui->bodyLgtInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->backHgtInfoLabel->setText("m");
    ui->backHgtInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->hipHgtInfoLabel->setText("m");
    ui->hipHgtInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->rumpLgtInfoLabel->setText("m");
    ui->rumpLgtInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    ui->chestDptInfoLabel->setText("m");
    ui->chestDptInfoLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    ptRatioBtnGroup = new QButtonGroup(this);
    ptRatioBtnGroup->addButton(ui->point1Ratio, 0);
    ptRatioBtnGroup->addButton(ui->point2Ratio, 1);
    ptRatioBtnGroup->addButton(ui->point3Ratio, 2);
    ptRatioBtnGroup->addButton(ui->point4Ratio, 3);
    ptRatioBtnGroup->addButton(ui->point5Ratio, 4);
    ptRatioBtnGroup->addButton(ui->point6Ratio, 5);
    ptRatioBtnGroup->addButton(ui->point7Ratio, 6);
    ptRatioBtnGroup->addButton(ui->point8Ratio, 7);
    ptRatioBtnGroup->button(0)->setChecked(true);

    ptCheckboxGroup = new QButtonGroup(this);
    ptCheckboxGroup->setExclusive(false);
    ptCheckboxGroup->addButton(ui->point1Checkbox, 0);
    ptCheckboxGroup->addButton(ui->point2Checkbox, 1);
    ptCheckboxGroup->addButton(ui->point3Checkbox, 2);
    ptCheckboxGroup->addButton(ui->point4Checkbox, 3);
    ptCheckboxGroup->addButton(ui->point5Checkbox, 4);
    ptCheckboxGroup->addButton(ui->point6Checkbox, 5);
    ptCheckboxGroup->addButton(ui->point7Checkbox, 6);
    ptCheckboxGroup->addButton(ui->point8Checkbox, 7);
    for (int i = 0; i < 8; i++)
    {
//        pointList[i].ifSaved = false;
        ptCheckboxGroup->button(i)->setCheckable(false);
        ptCheckboxGroup->button(i)->blockSignals(true);
    }
//    ptCheckboxGroup->blockSignals(true);

    // init points info list
    for (int i = 0; i < 8; i++)
    {
        ImgPoint ip(QPoint(0, 0), false, i);
//        ip.ratioBtnPtr = ptRatioBtnGroup->button(i);
//        ip.checkBoxPtr = ptCheckboxGroup->button(i);
        ip.markItemPtr = Q_NULLPTR;
        pointList.push_back(ip);
    }
//    spScene = new SelectPointScene(0);

    // 3d camera settings
//    ImgMarkScene *iks = new ImgMarkScene();
    ifm3dViewer.initViewer(ui->pclViewerWidget);

    // connects of 2d camera
//    connect(ui->OpenVideo, &QPushButton::clicked, this, &MainWindow::openLocal);
//    connect(ui->OpenCamera, &QPushButton::clicked, this, &MainWindow::openUrl);
    connect(ui->SnapShot, &QPushButton::clicked, this, &MainWindow::takeSnapShot);
    connect(ui->actionOpen_Video, &QAction::triggered, this, &MainWindow::openLocal);
    connect(ui->actionOpen_Camera, &QAction::triggered, this, &MainWindow::openUrl);
    connect(_player, &VlcMediaPlayer::end, _player, &VlcMediaPlayer::stop);
    connect(_player, &VlcMediaPlayer::error, this, &MainWindow::displayVlcError);

    // connects of compute body measurement
    connect(ui->imageTableWidget, &QTableWidget::itemClicked, this, &MainWindow::display2dImage);
    connect(ui->point1Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point2Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point3Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point4Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point5Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point6Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point7Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);
    connect(ui->point8Ratio, &QRadioButton::toggled, this, &MainWindow::toggleSelectedFlag);

    connect(imgMarkScene, &ImgMarkScene::pointInfo, this, &MainWindow::updatePointInfo);
    connect(ui->point1Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point2Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point3Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point4Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point5Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point6Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point7Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);
    connect(ui->point8Checkbox, &QCheckBox::clicked, this, &MainWindow::savePointInfo);

    connect(ui->point1Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint1Info);
    connect(ui->point2Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint2Info);
    connect(ui->point3Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint3Info);
    connect(ui->point4Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint4Info);
    connect(ui->point5Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint5Info);
    connect(ui->point6Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint6Info);
    connect(ui->point7Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint7Info);
    connect(ui->point8Checkbox, &QCheckBox::clicked, this, &MainWindow::unsavePoint8Info);

    connect(ui->resetButton, &QPushButton::clicked, this, &MainWindow::clearAll);
    connect(ui->cbmButton, &QPushButton::clicked, this, &MainWindow::computeBodyMeasurement);

    // connects of 3d camera
//    connect(ui->openPCD, &QPushButton::clicked, &ifm3dViewer, &IFM3DViewer::openLocal);
//    connect(ui->open3dCamera, &QPushButton::clicked, this, &MainWindow::open3dCamera);
//    connect(ui->snapShot3d, &QPushButton::clicked, this, &MainWindow::takeSnapShot3d);
    connect(ui->snapShot3d, &QPushButton::clicked, this ,&MainWindow::takeSnapShot);

    connect(ui->setThresholdAction, &QAction::triggered, this, &MainWindow::setBMScoreThreshold);
    connect(ui->helpButton, &QPushButton::clicked, this, &MainWindow::showSelectPointsHelp);

    connect(ui->open3DcameraAction, &QAction::triggered, this, &MainWindow::open3dCamera);
    connect(ui->openPCDAction, &QAction::triggered, &ifm3dViewer, &IFM3DViewer::openLocal);

    // menu action
    connect(ui->saveBMIAction, &QAction::triggered, this, &MainWindow::saveBMIToFile);
    connect(ui->aboutAppAction, &QAction::triggered, this, &MainWindow::showAboutInfo);
    connect(ui->aboutQtAction, &QAction::triggered, this, &MainWindow::showQtAbout);
    connect(ui->setSnapshotPathAction, &QAction::triggered, this, &MainWindow::setSanpshotPath);

    // Files Tab
    // init components of 2D component
    ui->imgFilesDisplayWidget->setStyleSheet("border:1px solid black");
    filesTabVlcInstance = new VlcInstance(VlcCommon::args(), this);
    filesTabVlcPlayer = new VlcMediaPlayer(filesTabVlcInstance);
    filesTabVlcPlayer->setVideoWidget(ui->imgFilesDisplayWidget);
    ui->imgFilesDisplayWidget->setMediaPlayer(filesTabVlcPlayer);
    ui->filesTW2DFilesList->setSelectionMode(QAbstractItemView::SingleSelection);

    connect(ui->filesTWOpenFileBtn, &QPushButton::clicked, this, &MainWindow::filesTabOpenFile);
    connect(filesTabVlcPlayer, &VlcMediaPlayer::end, filesTabVlcPlayer, &VlcMediaPlayer::stop);
    connect(ui->filesTWSnapshotBtn, &QPushButton::clicked, this, &MainWindow::filesTabVideoSnapshot);
    connect(ui->displayTabWidget, &QTabWidget::currentChanged, this, &MainWindow::disableFilesTabSSBtn);
    connect(ui->filesTW2DFilesList, &QListWidget::itemClicked, this, &MainWindow::files2DTabDisplayImg);

    // init components of 3D component
    fileTabCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    fileTab3DViewer.reset(new pcl::visualization::PCLVisualizer ("fileTab3DViewer", false));
    fileTab3DViewer->addPointCloud<pcl::PointXYZ>(fileTabCloud, "fileTabCloud");

    fileTab3DViewer->setBackgroundColor (0.1, 0.1, 0.1);
    fileTab3DViewer->addCoordinateSystem (1.0, "local_cloud", 0);
    ui->localPCDDisplayWidget->SetRenderWindow(fileTab3DViewer->getRenderWindow());
    fileTab3DViewer->setupInteractor(ui->localPCDDisplayWidget->GetInteractor(), ui->localPCDDisplayWidget->GetRenderWindow());
    ui->localPCDDisplayWidget->update();
    fileTab3DViewer->resetCamera();

    connect(ui->filesTW3DFilesList, &QListWidget::itemClicked, this, &MainWindow::files3DTabDisplayImg);
    connect(ui->filesTWSelectBtn, &QPushButton::clicked, this, &MainWindow::filesTabSelect);
}

MainWindow::~MainWindow()
{
    delete _player;
    delete _media;
    delete _instance;
    delete filesTabVlcPlayer;
    delete filesTabVlcMedia;
    delete filesTabVlcInstance;
    delete ui;
}

void MainWindow::openLocal()
{
    // clear player
    _player->stop();

    QString file =
            QFileDialog::getOpenFileName(this, tr("Open file"),
                                         QDir::homePath(),
                                         tr("Multimedia files(*.mp4 *.mkv *.dvi *.flv *.rmvb *.wmv *avi *wmv);;Image files(*.png *.jpg *.jpeg *.bmp *.gif)"));

    if (file.isEmpty())
        return;

    // remove specail process for image files
    disconnect(_player, &VlcMediaPlayer::playing, _player, &VlcMediaPlayer::pause);

    QFileInfo fi(file);
    QString fileType = fi.suffix();

    // if type is image, player pause!
    if (fileType == "jpg" || fileType == "png"
     || fileType == "jpeg" || fileType == "bmp"
     || fileType == "gif")
    {
        connect(_player, &VlcMediaPlayer::playing, _player, &VlcMediaPlayer::pause);
        vlc2DcameraSSThd.isImg = true;
    }
    else
    {
        vlc2DcameraSSThd.isImg = false;
    }
    _media = new VlcMedia(file, true, _instance);

    _player->open(_media);
}

void MainWindow::openUrl()
{
    QString url =
            QInputDialog::getText(this, tr("Open Url"), tr("Enter the URL you want to play"), QLineEdit::Normal, "rtsp://admin:*200wan*@192.168.1.2:554/MPEG-4/ch1/main/av_stream");

    if (url.isEmpty())
        return;

    ifConnect2DCam = true;

    _media = new VlcMedia(url, _instance);

    _player->open(_media);
}

void MainWindow::takeSnapShot()
{
    if (!ifConnect2DCam)
    {
        QMessageBox::warning(nullptr, "Warning", "2D camera has been connected. Please check 2D camera connection.");
        return;
    }
    if (!ifm3dViewer.getCameraState())
    {
        QMessageBox::warning(nullptr, "Warning", "3D camera has been connected. Please check 3D camera connection.");
        return;
    }
    // Pass player to snapshot thread
    vlc2DcameraSSThd.takeSnapshot(_player);

    // record name of 2D snapshot
    QString img2d = vlc2DcameraSSThd.getSnapshotName();
    image2DName = QFileInfo(img2d).fileName();

    // take 3D snapshot
    ifm3dViewer.takeSnapshot();

    // record name of 3D snapshot
    QString img3d = ifm3dViewer.getSnapshotName();
    image3DName = QFileInfo(img3d).fileName();

    computeGroupList.push_back(QPair<QString, QString>(img2d, img3d));

    addData2Table();
}

void MainWindow::open3dCamera()
{
    QString ip_add =
            QInputDialog::getText(this, tr("Open 3D Camera"), tr("Enter the IP address of your camera"), QLineEdit::Normal, "192.168.1.3");
    if (ip_add.isEmpty())
        return;
    ifConnect3DCam = true;

    ifm3dViewer.openCamera(ip_add);
}

void MainWindow::takeSnapShot3d()
{
    // take 3D snapshot
    ifm3dViewer.takeSnapshot();

    // record name of 3D snapshot
    QFileInfo fi(ifm3dViewer.getSnapshotName());
    image3DName = fi.fileName();

    // add data into table
    if (image2DName.isEmpty())
        return;
    if (image3DName.isEmpty())
        return;
    addData2Table();
}

void MainWindow::add2DImage2Table()
{
    dataCount++;
//    imgName = vlc2DcameraSSThd.ssname;

//    QString imageName = QFileDialog::getOpenFileName(this, tr("Open file"),
//                                             ".",
//                                             tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
//    QFileInfo fi(imageName);
//    QLabel imgLabel;
//    imgLabel.setText(fi.fileName());

//    ui->imageTableWidget->addItem(imageName);
//    ui->imageTableWidget->insertRow(0);
//    qDebug() << ui->imageTableWidget->count();
}

void MainWindow::removeImage()
{
//    QListWidgetItem *curItem = ui->imageTableWidget->currentItem();
//    if (curItem == nullptr)
//    {
//        QMessageBox::information(nullptr, tr("Info"), tr("There are no item selected!"));
//        return;
//    }
//    ui->imageTableWidget->removeItemWidget(ui->imageTableWidget->currentItem());
//    delete ui->imageTableWidget->currentItem();
}

void MainWindow::display2dImage()
{
    clearAll();
    imgMarkScene->clear();
    for (int i = 0; i < 8; i++)
    {
//        pointList[i].ifSaved = false;
        ptCheckboxGroup->button(i)->setCheckable(true);
        ptCheckboxGroup->button(i)->blockSignals(false);
    }

//    QString appPath = qApp->applicationDirPath();
//    QString imagePath = QFileInfo(snapshotPath).filePath() + "/" + ui->imageTableWidget->selectedItems().at(0)->text();
    QString imagePath = computeGroupList[ui->imageTableWidget->currentRow()].first;

    if (imagePath.isEmpty())
        return;

    cv::Mat cvImg = cv::imread(imagePath.toLocal8Bit().toStdString());
    QImage displayedImg;
    cv::Mat tempRgb;
    // Convert Mat BGR to QImage RGB
    if (cvImg.channels() == 3)
    {
        cv::cvtColor(cvImg, tempRgb, CV_BGR2RGB);
        displayedImg = QImage((const unsigned char*)(tempRgb.data),
                                tempRgb.cols, tempRgb.rows,
                                tempRgb.cols * tempRgb.channels(),
                                QImage::Format_RGB888);
    }
    else
    {
        displayedImg = QImage((const unsigned char*)(cvImg.data),
                                cvImg.cols, cvImg.rows,
                                cvImg.cols * cvImg.channels(),
                                QImage::Format_Indexed8);
    }

    QPixmap showedPixImg = QPixmap::fromImage(displayedImg);
    imgMarkScene->addPixmap(showedPixImg);
    imgMarkScene->setSceneRect(showedPixImg.rect());
    ui->imageGraphicsView->setScene(imgMarkScene);
    ui->imageGraphicsView->fitInView(imgMarkScene->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void MainWindow::wheelEvent(QWheelEvent *event)
{
    if(QApplication::keyboardModifiers() == Qt::ControlModifier)
    {
        if(event->delta() > 0)
        {
            ui->imageGraphicsView->scale(1.1, 1.1);
        }
        else
        {
            ui->imageGraphicsView->scale(0.9, 0.9);
        }
    }
}

void MainWindow::adjustImageTableSize()
{

}

void MainWindow::addData2Table()
{
    dataCount = dataCount + 1;
    int iTRowCount = ui->imageTableWidget->rowCount();
    ui->imageTableWidget->insertRow(iTRowCount);

    QTableWidgetItem *img2DItem = new QTableWidgetItem(image2DName);
    QTableWidgetItem *img3DItem = new QTableWidgetItem(image3DName);

    ui->imageTableWidget->setItem(iTRowCount, 0, img2DItem);
    ui->imageTableWidget->setItem(iTRowCount, 1, img3DItem);
}

void MainWindow::toggleSelectedFlag()
{
    imgMarkScene->setSelectedFlag(true);
    imgMarkScene->curItemID = ptRatioBtnGroup->checkedId() + 1;
}

void MainWindow::updatePointInfo(qreal x, qreal y)
{
    curPos = QPoint(x, y);
    useDefalutValue = false;
    qDebug() << "Current position: " << "[" << x << "," << y << "]";
}

void MainWindow::savePointInfo(bool checked)
{
    int curRatioID = ptRatioBtnGroup->checkedId();
    int curCheckboxID = ptCheckboxGroup->checkedId();
    if (checked)
    {
        if (curRatioID != curCheckboxID ||
                imgMarkScene->getPrevItem() == Q_NULLPTR)
        {
            QMessageBox::warning(nullptr, tr("Warning"), tr("Point ") + QString::number(curCheckboxID + 1, 10) +
                                 tr(" has not been set position!"));
            ptCheckboxGroup->button(curCheckboxID)->setChecked(false);
            return;
        }
        else
        {
            pointList[curRatioID].ifSaved = true;
            imgMarkScene->pointSaveds[curRatioID] = true;
            pointList[curRatioID].pos = curPos;
            pointList[curRatioID].markItemPtr = imgMarkScene->getPrevItem();
            if (checkIfAllSaved())
            {
                imgMarkScene->setAllSaved();
            }
            imgMarkScene->resetPrevItem();
            int nextID = curRatioID + 1;
            if (nextID == 8)    nextID = 0;
            ptRatioBtnGroup->button(nextID)->setChecked(true);
        }
    }
}

void MainWindow::unsavePoint1Info(bool checked)
{
    int curCheckboxID = 0;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint2Info(bool checked)
{
    int curCheckboxID = 1;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint3Info(bool checked)
{
    int curCheckboxID = 2;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint4Info(bool checked)
{
    int curCheckboxID = 3;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint5Info(bool checked)
{
    int curCheckboxID = 4;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint6Info(bool checked)
{
    int curCheckboxID = 5;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint7Info(bool checked)
{
    int curCheckboxID = 6;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::unsavePoint8Info(bool checked)
{
    int curCheckboxID = 7;
    if (!checked)
    {
        pointList[curCheckboxID].ifSaved = false;
        imgMarkScene->pointSaveds[curCheckboxID] = false;
        if (pointList[curCheckboxID].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[curCheckboxID].markItemPtr);
            pointList[curCheckboxID].markItemPtr = Q_NULLPTR;
        }
        imgMarkScene->resetAllSaved();
        ptRatioBtnGroup->button(curCheckboxID)->setChecked(true);
    }
}

void MainWindow::clearAll()
{
    // unset ifComputed flag
    ifComputed = false;

    useDefalutValue = true;
    // TODO: check if previous BMI result is saved

    // clear items in graphics view
    for (int i = 0; i < 8; i++)
    {
        if (pointList[i].markItemPtr != Q_NULLPTR)
        {
            imgMarkScene->removeItem(pointList[i].markItemPtr);
            pointList[i].markItemPtr = Q_NULLPTR;
        }
    }

    if (imgMarkScene->getPrevItem() != Q_NULLPTR)
        imgMarkScene->removeItem(imgMarkScene->getPrevItem());
    imgMarkScene->resetPrevItem();

    // clear info of points
    ptRatioBtnGroup->button(0)->setChecked(true);
    for (int i = 0; i < 8; i++)
    {

        pointList[i].ifSaved = false;
        imgMarkScene->pointSaveds[i] = false;
        ptCheckboxGroup->button(i)->setChecked(false);
    }
    imgMarkScene->resetAllSaved();

    // clear body measurement info
    ui->waistHgtInfoLabel->setText("m");
    ui->withersHgtInfoLabel->setText("m");
    ui->bodyLgtInfoLabel->setText("m");
    ui->backHgtInfoLabel->setText("m");
    ui->hipHgtInfoLabel->setText("m");
    ui->rumpLgtInfoLabel->setText("m");
    ui->chestDptInfoLabel->setText("m");
}

bool MainWindow::checkIfAllSaved()
{
    for (int i = 0; i < 8; i++)
    {
        if (!pointList[i].ifSaved)
            return false;
    }
    return true;
}

void MainWindow::computeBodyMeasurement()
{
    if (!useDefalutValue && !checkIfAllSaved())
    {
        QMessageBox::warning(nullptr, tr("Warning"), tr("There is no image group selected, or there are points un-set positon. Please select a image group or set position for all points."));
        return;
    }

    // set ifComputed flag
    ifComputed = true;

    // step 1: init BMS core
    bmscore.initBMScore();

    // step 2: input pcd data
//    QString pcdPath = QFileInfo(snapshotPath).filePath() + "/" + ui->imageTableWidget->selectedItems().at(1)->text();
    QString pcdPath = computeGroupList[ui->imageTableWidget->currentRow()].second;
    bool readPCDSuccess = bmscore.readCloudData(pcdPath.toStdString());
    if (!readPCDSuccess)
    {
        qDebug() << "Read PCD file failed!";
        return;
    }

    // step 3: set positions of 8 points
    std::vector<PtPos> pps(8);
    qDebug() << "Running to line: " << __LINE__;
    for (int i = 0; i < 8; i++)
    {
        pps[i].x = pointList[i].pos.x();
        pps[i].y = pointList[i].pos.y();
    }
    qDebug() << "Running to line: " << __LINE__;
    bmscore.setPtPosList(pps);
    qDebug() << "Set Positon success: " << __LINE__;

    // step 4: compute body measurent according to PCD data and points data
    bmscore.computeBodyMeasurement();

    // display result in GUI
    ui->withersHgtInfoLabel->setText(QString("%1").arg(bmscore.getWithersHeight()) + " m");
    ui->chestDptInfoLabel->setText(QString("%1").arg(bmscore.getChestDepth()) + " m");
    ui->backHgtInfoLabel->setText(QString("%1").arg(bmscore.getBackHeight()) + " m");
    ui->waistHgtInfoLabel->setText(QString("%1").arg(bmscore.getWaistHeight()) + " m");
    ui->hipHgtInfoLabel->setText(QString("%1").arg(bmscore.getHipHeight()) + " m");
    ui->rumpLgtInfoLabel->setText(QString("%1").arg(bmscore.getRumpLength()) + " m");
    ui->bodyLgtInfoLabel->setText(QString("%1").arg(bmscore.getBodyLength()) + " m");
}

void MainWindow::setBMScoreThreshold()
{
    bool ok = false;
    int curValue = bmscore.getThreshold();
    int t = QInputDialog::getInt(this, "Set Threshold", "Please set value of threshold: ", curValue, 5, 100, 1, &ok);
    if (!ok)
        return;

    bmscore.setThreshold(t);
}

void MainWindow::showSelectPointsHelp()
{
    HelpDialog *hdlg = new HelpDialog;
    hdlg->setModal(false);
    hdlg->show();
}

QString MainWindow::getFileNamePrefix()
{
    return QString();
}

void MainWindow::saveBMIToFile()
{
    // check if has computed result
    if (!ifComputed)
    {
        QMessageBox::warning(nullptr, "Warning", "There are no result to save!");
        return;
    }

    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    QString defaultFilePath = QDir::homePath() + "/" + current_date + "-BMI-result.txt";
    QString realFilePath = QFileDialog::getSaveFileName(nullptr, "Save BMI Computation Result",
                                                    defaultFilePath);

    if (realFilePath.isEmpty())
        return;

    QFile file(realFilePath);

    if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
        return;

    QTextStream out(&file);
    out << "======== Body Measurement System Computation Result ========";
    out << "\n";
    out << "-- Images Info --\n";
    out << "\n";
    out << "2D Image File: " << image2DName << "\n";
    out << "3D Image File: " << image3DName << "\n";
    out << "\n";
    out << "-- Position Info --\n";
    out << "\n";
    for (int i = 0; i < 8; i++)
        out << "Point " << i << ": (" << pointList[i].pos.x() << ", " << pointList[i].pos.y() << ")\n";
    out << "\n";
    out << "-- Body Measurement Info --\n";
    out << "\n";

    out << "Withers Height: " << bmscore.getWithersHeight() << " m\n";
    out << "Chest Depth: " << bmscore.getChestDepth() << " m\n";
    out << "Back Height: " << bmscore.getBackHeight() << " m\n";
    out << "Body Length: " << bmscore.getBodyLength() << " m\n";
    out << "Waist Height: " << bmscore.getWaistHeight() << " m\n";
    out << "Rump Length: " << bmscore.getRumpLength() << " m\n";
    out << "Hip Height: " << bmscore.getHipHeight() << " m\n";

    file.close();
}

void MainWindow::showAboutInfo()
{
    QMessageBox::about(nullptr, "About QinchuanCattleBMS v1.0", "This is a software to compute body measurement of Cattles. This software is licensed under the <a href=https://opensource.org/licenses/GPL-3.0>GNU General Public License v3.0</a>. You can report bugs <a href=https://github.com/chenyangcheung/QinchuanCattleBMS/issues>here</a> to us.");
    return;
}

void MainWindow::showQtAbout()
{
    QMessageBox::aboutQt(nullptr, "About Qt");
    return;
}

void MainWindow::setSanpshotPath()
{
    QString ssp =
            QInputDialog::getText(this, tr("Set sanpshot path"), tr("Enter the path to save snapshot"), QLineEdit::Normal, snapshotPath);

    if (ssp.isEmpty())
        return;

    snapshotPath = ssp;
    if (!QDir().exists(ssp))
    {
        bool r = QDir().mkpath(ssp);
        // to save error to log file
    }
    // TODO: consider immigranting previous images

    ifm3dViewer.setSnapshotPath(ssp);
    vlc2DcameraSSThd.setSnapshotPath(ssp);
}

void MainWindow::displayVlcError()
{
    QMessageBox::warning(nullptr, "Warning", "Could not open IP Camera! Please check your url.");
    return;
}

// Files Tab
void MainWindow::filesTabOpenFile()
{
    if (ui->displayTabWidget->currentIndex() == 0)
    {
        // clear player
        filesTabVlcPlayer->stop();

        QString file =
                QFileDialog::getOpenFileName(this, tr("Open file"),
                                             QDir::homePath(),
                                             tr("Multimedia files(*.mp4 *.mkv *.dvi *.flv *.rmvb *.wmv *avi *wmv);;Image files(*.png *.jpg *.jpeg *.bmp *.gif)"));

        if (file.isEmpty())
            return;
        filesTabVlcSSThd.isImg = false;

        // remove specail process for image files
        disconnect(filesTabVlcPlayer, &VlcMediaPlayer::playing, filesTabVlcPlayer, &VlcMediaPlayer::pause);

        QFileInfo fi(file);
        QString fileType = fi.suffix();

        // if type is image, player pause!
        if (fileType == "jpg" || fileType == "png"
         || fileType == "jpeg" || fileType == "bmp"
         || fileType == "gif")
        {
            connect(filesTabVlcPlayer, &VlcMediaPlayer::playing, filesTabVlcPlayer, &VlcMediaPlayer::pause);
            ui->filesTW2DFilesList->addItem(fi.fileName());
            ui->filesTWSnapshotBtn->setEnabled(false);
            files2DList.push_back(file);
        }
        else
        {
            ui->filesTWSnapshotBtn->setEnabled(true);
        }

        ui->displayTabWidget->setCurrentIndex(0);
        filesTabVlcMedia = new VlcMedia(file, true, filesTabVlcInstance);

        filesTabVlcPlayer->open(filesTabVlcMedia);
    }
    else
    {
        QString filename = QFileDialog::getOpenFileName(nullptr,
                   tr("Open PointCloud"), QDir::homePath(), tr("Open PCD files (*.pcd)"));
        if (!filename.isEmpty())
        {
            ui->displayTabWidget->setCurrentIndex(1);
            // add into list
            files3DList.push_back(filename);
            ui->filesTW3DFilesList->addItem(QFileInfo(filename).fileName());
            std::string file_name = filename.toStdString();
            pcl::PCLPointCloud2 cloud2;
            Eigen::Vector4f origin;
            Eigen::Quaternionf orientation;
            int pcd_version;
            int data_type;
            unsigned int data_idx;
            pcl::PCDReader rd;
            rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

            if (data_type == 0)
            {
                pcl::io::loadPCDFile(filename.toStdString(), *fileTabCloud);
            }
            else if (data_type == 2)
            {
                pcl::PCDReader reader;
                reader.read<pcl::PointXYZ>(filename.toStdString(), *fileTabCloud);
            }
            files3DList.push_back(filename);
            fileTab3DViewer->updatePointCloud<pcl::PointXYZ>(fileTabCloud, "fileTabCloud");
            fileTab3DViewer->resetCamera();
            ui->localPCDDisplayWidget->update();
        }
    }
}

void MainWindow::filesTabDelete()
{
    int id2d = ui->filesTW2DFilesList->currentRow();
    int id3d = ui->filesTW3DFilesList->currentRow();

    if (id2d != -1)
    {
        QString filename = files2DList[id2d];
        ui->filesTW2DFilesList->removeItemWidget(ui->filesTW2DFilesList->currentItem());
        files2DList.remove(id2d);
    }
    if (id2d != -1)
    {
        QString filename = files3DList[id3d];
        ui->filesTW3DFilesList->removeItemWidget(ui->filesTW3DFilesList->currentItem());
        files3DList.remove(id3d);
    }
}

void MainWindow::filesTabSelect()
{
    int id2d = ui->filesTW2DFilesList->currentRow();
    if (id2d == -1)
    {
        QMessageBox::warning(nullptr, "Warning", "2D File has not been selected. Please select a 2D file from 2D File List.");
        return;
    }

    int id3d = ui->filesTW3DFilesList->currentRow();
    if (id3d == -1)
    {
        QMessageBox::warning(nullptr, "Warning", "3D File has not been selected. Please select a 3D file from 3D File List.");
        return;
    }


    image2DName = QFileInfo(files2DList[id2d]).fileName();
    image3DName = QFileInfo(files3DList[id3d]).fileName();

    computeGroupList.push_back(QPair<QString, QString>(files2DList[id2d], files3DList[id3d]));
    addData2Table();
}

void MainWindow::filesTabVideoSnapshot()
{
    // Pass player to snapshot thread
    filesTabVlcSSThd.takeSnapshot(filesTabVlcPlayer);
    qDebug() << filesTabVlcSSThd.getSnapshotName();
    // TODO: check if get snapshot
    files2DList.push_back(filesTabVlcSSThd.getSnapshotName());
    ui->filesTW2DFilesList->addItem(QFileInfo(filesTabVlcSSThd.getSnapshotName()).fileName());
}

void MainWindow::disableFilesTabSSBtn(int index)
{
    if (index == 1)
    {
        ui->filesTWSnapshotBtn->setEnabled(false);
    }
    else
    {
        ui->filesTWSnapshotBtn->setEnabled(true);
    }
}

void MainWindow::files2DTabDisplayImg()
{
//    QString item2DText = ui->filesTW2DFilesList->currentItem()->text();
    int curId = ui->filesTW2DFilesList->currentRow();
    QString filePath = files2DList[curId];

    ui->displayTabWidget->setCurrentIndex(0);
    // clear player
    _player->stop();
    connect(filesTabVlcPlayer, &VlcMediaPlayer::playing, filesTabVlcPlayer, &VlcMediaPlayer::pause);
    ui->filesTWSnapshotBtn->setEnabled(false);
    filesTabVlcMedia = new VlcMedia(filePath, true, filesTabVlcInstance);
    filesTabVlcPlayer->open(filesTabVlcMedia);
}

void MainWindow::files3DTabDisplayImg()
{
//    QString item3DText = ui->filesTW3DFilesList->currentItem()->text();
    int curId = ui->filesTW3DFilesList->currentRow();

    QString filePath = files3DList[curId];
    std::string file_name = filePath.toStdString();
    pcl::PCLPointCloud2 cloud2;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int pcd_version;
    int data_type;
    unsigned int data_idx;
    pcl::PCDReader rd;

    ui->displayTabWidget->setCurrentIndex(1);

    rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

    if (data_type == 0)
    {
        pcl::io::loadPCDFile(file_name, *fileTabCloud);
    }
    else if (data_type == 2)
    {
        pcl::PCDReader reader;
        reader.read<pcl::PointXYZ>(file_name, *fileTabCloud);
    }

    fileTab3DViewer->updatePointCloud<pcl::PointXYZ>(fileTabCloud, "fileTabCloud");
    fileTab3DViewer->resetCamera();
    ui->localPCDDisplayWidget->update();
}
