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

    // 2d camera settings
    _instance = new VlcInstance(VlcCommon::args(), this);
    _player = new VlcMediaPlayer(_instance);
    _player->setVideoWidget(ui->camera);
    ui->camera->setMediaPlayer(_player);

    // 2d image settings
//    ui->imageListWidget->addItem("test");

    // 3d camera settings
    ifm3dViewer.initViewer(ui->pclViewerWidget);

    // connects of 2d camera
    connect(ui->OpenVideo, &QPushButton::clicked, this, &MainWindow::openLocal);
    connect(ui->OpenCamera, &QPushButton::clicked, this, &MainWindow::openUrl);
    connect(ui->SnapShot, &QPushButton::clicked, this, &MainWindow::takeSnapShot);
    connect(ui->actionOpen_Video, &QAction::triggered, this, &MainWindow::openLocal);
    connect(ui->actionOpen_Camera, &QAction::triggered, this, &MainWindow::openUrl);

    // connects of 2d image
    connect(ui->addImgButton, &QPushButton::clicked, this, &MainWindow::addImage);
    connect(ui->imageListWidget, &QListWidget::itemActivated, this, &MainWindow::display2dImage);
    connect(ui->rmImgButton, &QPushButton::clicked, this, &MainWindow::removeImage);

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
                                         QDir::homePath(),
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
    ifm3dViewer.takeSnapshot();
}

void MainWindow::addImage()
{
    QString imageName = QFileDialog::getOpenFileName(this, tr("Open file"),
                                             QDir::homePath(),
                                             tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
    QFileInfo fi(imageName);
    QLabel imgLabel;
    imgLabel.setText(fi.fileName());

    ui->imageListWidget->addItem(imageName);
}

void MainWindow::removeImage()
{
    QListWidgetItem *curItem = ui->imageListWidget->currentItem();
    if (curItem == nullptr)
    {
        QMessageBox::information(nullptr, tr("Info"), tr("There are no item selected!"));
        return;
    }
    ui->imageListWidget->removeItemWidget(ui->imageListWidget->currentItem());
    delete ui->imageListWidget->currentItem();
}

void MainWindow::display2dImage()
{
    QString imagePath = ui->imageListWidget->currentItem()->text();

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
//    QImage *displayedImg = new QImage(imagePath);
    QPixmap showedPixImg = QPixmap::fromImage(displayedImg);
    imageScene = new QGraphicsScene(this);
    imageScene->addPixmap(showedPixImg);
    imageScene->setSceneRect(showedPixImg.rect());
    ui->imageGraphicsView->setScene(imageScene);
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
