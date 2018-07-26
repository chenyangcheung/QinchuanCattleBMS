#include "snapshotthread.h"
#include <QDateTime>
#include <QApplication>
#include <QDebug>

SnapshotThread::SnapshotThread(QObject *parent)
    : QThread(parent)
{

}

void SnapshotThread::run()
{
    bool ifsuccess = _video->takeSnapshot(ssname);
    qDebug() << "Snap shot result: " << ifsuccess;
}

void SnapshotThread::takeSnapshot(VlcMediaPlayer *&player)
{
    _video = new VlcVideo(player);
    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    // pass image path to slot function -- VlcMediaPlayer::snapShotTaken
    ssname = qApp->applicationDirPath() + "/" + current_date + ".jpg";
//    qDebug() << "pass file name: " + ssname;
    start();
}
