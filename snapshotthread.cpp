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
    bool success = _video->takeSnapshot(ssname);
    if (success)
        qDebug() << "Get snapshot: " << ssname;
    delete _video;
}

void SnapshotThread::takeSnapshot(VlcMediaPlayer *&player)
{
    // Create a video environment
    // and don't forget delete after using it
    _video = new VlcVideo(player);

    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    ssname = qApp->applicationDirPath() + "/" + current_date + ".jpg";

    // start to take snapshot
    start();
}
