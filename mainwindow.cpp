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


    QStringList tableHeader;

    tableHeader << tr("2D Images") << tr("3D Images");
    ui->imageTableWidget->setHorizontalHeaderLabels(tableHeader);

    imgMarkScene = new ImgMarkScene(this);

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
    connect(ui->OpenVideo, &QPushButton::clicked, this, &MainWindow::openLocal);
    connect(ui->OpenCamera, &QPushButton::clicked, this, &MainWindow::openUrl);
    connect(ui->SnapShot, &QPushButton::clicked, this, &MainWindow::takeSnapShot);
    connect(ui->actionOpen_Video, &QAction::triggered, this, &MainWindow::openLocal);
    connect(ui->actionOpen_Camera, &QAction::triggered, this, &MainWindow::openUrl);

    // connects of compute body measurement
    connect(ui->imageTableWidget, &QTableWidget::itemActivated, this, &MainWindow::display2dImage);
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
    connect(ui->openPCD, &QPushButton::clicked, &ifm3dViewer, &IFM3DViewer::openLocal);
    connect(ui->open3dCamera, &QPushButton::clicked, this, &MainWindow::open3dCamera);
    connect(ui->snapShot3d, &QPushButton::clicked, this, &MainWindow::takeSnapShot3d);

    connect(ui->setThresholdAction, &QAction::triggered, this, &MainWindow::setBMScoreThreshold);
    connect(ui->helpButton, &QPushButton::clicked, this, &MainWindow::showSelectPointsHelp);
}

MainWindow::~MainWindow()
{
    delete _player;
    delete _media;
    delete _instance;
    delete ui;
}

void MainWindow::openLocal()
{
    QString file =
            QFileDialog::getOpenFileName(this, tr("Open file"),
                                         ".",
                                         tr("Multimedia files(*)"));

    if (file.isEmpty())
        return;

    _media = new VlcMedia(file, true, _instance);

    _player->open(_media);
}

void MainWindow::openUrl()
{
    QString url =
            QInputDialog::getText(this, tr("Open Url"), tr("Enter the URL you want to play"), QLineEdit::Normal, "rtsp://admin:*200wan*@172.29.23.12:554/MPEG-4/ch1/main/av_stream");

    if (url.isEmpty())
        return;

    // TODO: add exception catch
    _media = new VlcMedia(url, _instance);

    _player->open(_media);
}

void MainWindow::takeSnapShot()
{
    // Pass player to snapshot thread
    SnapshotThread.takeSnapshot(_player);

    // record name of 2D snapshot
    QFileInfo fi(SnapshotThread.getSnapshotName());
    image2DName = fi.fileName();
}

void MainWindow::open3dCamera()
{
    QString ip_add =
            QInputDialog::getText(this, tr("Open 3D Camera"), tr("Enter the IP address of your camera"), QLineEdit::Normal, "169.254.58.137");
    if (ip_add.isEmpty())
        return;
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
//    imgName = SnapshotThread.ssname;

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

    QString appPath = qApp->applicationDirPath();
    QString imagePath = appPath + "/" + ui->imageTableWidget->selectedItems().at(0)->text();

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
    if (!checkIfAllSaved())
    {
        QMessageBox::warning(nullptr, tr("Warning"), tr("There are points un-set positon, please set position for them!"));
        return;
    }

    // step 1: init BMS core
    bmscore.initBMScore();

    // step 2: input pcd data
    QString pcdPath = qApp->applicationDirPath() + "/" + ui->imageTableWidget->selectedItems().at(1)->text();
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
    ui->withersHgtInfoLabel->setText(QString("%1").arg(bmscore.getWithersHeight()));
    ui->chestDptInfoLabel->setText(QString("%1").arg(bmscore.getChestDepth()));
    ui->backHgtInfoLabel->setText(QString("%1").arg(bmscore.getBackHeight()));
    ui->waistHgtInfoLabel->setText(QString("%1").arg(bmscore.getWaistHeight()));
    ui->hipHgtInfoLabel->setText(QString("%1").arg(bmscore.getHipHeight()));
    ui->rumpLgtInfoLabel->setText(QString("%1").arg(bmscore.getRumpLength()));
    ui->bodyLgtInfoLabel->setText(QString("%1").arg(bmscore.getBodyLength()));
}

void MainWindow::setBMScoreThreshold()
{
    bool ok = false;
    int t = QInputDialog::getInt(this, "Set Threshold", "Please set value of threshold: ", 10, 5, 100, 1, &ok);
    if (!ok)
        return;

    bmscore.setThreshold(t);
}

void MainWindow::showSelectPointsHelp()
{
    HelpDialog hdlg;
    hdlg.exec();
}
