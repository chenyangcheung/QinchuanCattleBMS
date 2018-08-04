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
//    ui->imageTableWidget->setColumnWidth(0, 10);
//    ui->imageTableWidget->setFixedWidth();

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

    // 3d camera settings
    ifm3dViewer.initViewer(ui->pclViewerWidget);

    // connects of 2d camera
    connect(ui->OpenVideo, &QPushButton::clicked, this, &MainWindow::openLocal);
    connect(ui->OpenCamera, &QPushButton::clicked, this, &MainWindow::openUrl);
    connect(ui->SnapShot, &QPushButton::clicked, this, &MainWindow::takeSnapShot);
    connect(ui->actionOpen_Video, &QAction::triggered, this, &MainWindow::openLocal);
    connect(ui->actionOpen_Camera, &QAction::triggered, this, &MainWindow::openUrl);

    // connects of 2d image
//    connect(ui->addImgButton, &QPushButton::clicked, this, &MainWindow::addImage);
//    connect(ui->imageTableWidget, &QTableWidget::itemActivated, this, &MainWindow::display2dImage);
//    connect(ui->rmImgButton, &QPushButton::clicked, this, &MainWindow::removeImage);
    connect(ui->imageTableWidget, &QTableWidget::itemActivated, this, &MainWindow::display2dImage);

    // connects of 3d camera
    connect(ui->openPCD, &QPushButton::clicked, &ifm3dViewer, &IFM3DViewer::openLocal);
    connect(ui->open3dCamera, &QPushButton::clicked, this, &MainWindow::open3dCamera);
    connect(ui->snapShot3d, &QPushButton::clicked, this, &MainWindow::takeSnapShot3d);
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
    QString appPath = qApp->applicationDirPath();
    QString imagePath = appPath + "/" + ui->imageTableWidget->selectedItems().at(0)->text();
    qDebug() << imagePath;
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
    imageScene = new QGraphicsScene(this);
    imageScene->addPixmap(showedPixImg);
    imageScene->setSceneRect(showedPixImg.rect());
    ui->imageGraphicsView->setScene(imageScene);
    ui->imageGraphicsView->fitInView(imageScene->itemsBoundingRect(), Qt::KeepAspectRatio);
    ui->imageGraphicsView->show();
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
//    imageScene->addEllipse()
}

void MainWindow::addData2Table()
{
    dataCount = dataCount + 1;
    int iTRowCount = ui->imageTableWidget->rowCount();
    ui->imageTableWidget->insertRow(iTRowCount);
//    QTableWidgetItem *idItem = new QTableWidgetItem(QString::number(dataCount, 10));
    QTableWidgetItem *img2DItem = new QTableWidgetItem(image2DName);
    QTableWidgetItem *img3DItem = new QTableWidgetItem(image3DName);

//    ui->imageTableWidget->setItem(iTRowCount, 0, idItem);
    ui->imageTableWidget->setItem(iTRowCount, 0, img2DItem);
    ui->imageTableWidget->setItem(iTRowCount, 1, img3DItem);
}
