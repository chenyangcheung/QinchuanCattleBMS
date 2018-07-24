#include <QFileDialog>
#include <QInputDialog>

#include <VLCQtCore/Common.h>
#include <VLCQtCore/Instance.h>
#include <VLCQtCore/Media.h>
#include <VLCQtCore/MediaPlayer.h>

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _media(0)
//    _equalizerDialog(new EqualizerDialog(this))
{
    ui->setupUi(this);
    _instance = new VlcInstance(VlcCommon::args(), this);
    _player = new VlcMediaPlayer(_instance);
    _player->setVideoWidget(ui->camera);
//    _equalizerDialog->setMediaPlayer(_player);
    ui->camera->setMediaPlayer(_player);
//    ui->camera->
    connect(ui->SnapShot, &QPushButton::clicked, this, &MainWindow::saveSnapShot);

}

MainWindow::~MainWindow()
{
    delete _player;
    delete _media;
    delete _instance;
    delete ui;
}

//void MainWindow::openLocal()
//{
//    QString file =
//            QFileDialog::getOpenFileName(this, tr("Open file"),
//                                         QDir::homePath(),
//                                         tr("Multimedia files(*)"));

//    if (file.isEmpty())
//        return;

//    _media = new VlcMedia(file, true, _instance);

//    _player->open(_media);
//}

//void MainWindow::openUrl()
//{
//    QString url =
//            QInputDialog::getText(this, tr("Open Url"), tr("Enter the URL you want to play"));

//    if (url.isEmpty())
//        return;

//    _media = new VlcMedia(url, _instance);

//    _player->open(_media);
//}


void MainWindow::on_OpenFile_clicked()
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

void MainWindow::on_OpenCamera_clicked()
{
    QString url =
            QInputDialog::getText(this, tr("Open Url"), tr("Enter the URL you want to play"));

    if (url.isEmpty())
        return;

    _media = new VlcMedia(url, _instance);

    _player->open(_media);
}

void MainWindow::on_pushButton_clicked()
{
//    QSize curSize = ui->camera->size();
//    const QRect R(QPoint(0, 0));
//    QPixmap record;
//    record = ui->camera->grab(QRect(QPoint(0, 0), QSize(ui->camera->width(), ui->camera->height())));
//    QString imagePath = QFileDialog::getSaveFileName(this,
//                                                     tr("Save File"),
//                                                     ".",
//                                                     tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
//    if (!imagePath.isEmpty())
//    {
////        cv::imwrite(imagePath.toStdString(), savedImage);
//        QImage img = record.toImage();
//        img.save(imagePath);
//    }
}

void MainWindow::saveSnapShot()
{
        QString imagePath = QFileDialog::getSaveFileName(this,
                                                         tr("Save File"),
                                                         ".",
                                                         tr("Image Files (*.png *.jpg *.jpeg *.bmp)"));
//        if (!imagePath.isEmpty())
//        {
//    //        cv::imwrite(imagePath.toStdString(), savedImage);
//            QImage img = record.toImage();
//            img.save(imagePath);
//        }
        // TODO: emit custom signl
        // pass image path to slot function -- VlcMediaPlayer::snapShotTaken
}

void MainWindow::TakeSnapShot()
{
    // TODO: Generate image name according to current time

    // TODO: emit custom signl
    // pass image path to slot function -- VlcMediaPlayer::snapShotTaken

}
