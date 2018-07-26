#include <QFileDialog>
#include <QInputDialog>
#include <QString>
#include <QDateTime>
#include <QDebug>

#include <VLCQtCore/Common.h>
#include <VLCQtCore/Instance.h>
#include <VLCQtCore/Media.h>
#include <VLCQtCore/MediaPlayer.h>
#include <VLCQtCore/Video.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _media(0)
{
    ui->setupUi(this);
    _instance = new VlcInstance(VlcCommon::args(), this);
    _player = new VlcMediaPlayer(_instance);
    _player->setVideoWidget(ui->camera);
    ui->camera->setMediaPlayer(_player);

    connect(ui->SnapShot, &QPushButton::clicked, this, &MainWindow::TakeSnapShot);
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

void MainWindow::TakeSnapShot()
{
//    qDebug() << "take snapshoting...";
    SnapshotThread.takeSnapshot(_player);
}
