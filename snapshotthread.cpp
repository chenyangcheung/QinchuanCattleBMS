#include "snapshotthread.h"
#include <QDateTime>
#include <QApplication>
#include <QDebug>
#include <QMessageBox>

SnapshotThread::SnapshotThread(QObject *parent)
    : QThread(parent)
{

}

void SnapshotThread::run()
{
    ifGetSnapshot = _video->takeSnapshot(ssname);
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

    // wait snapshot thread to quit
    quit();
    wait();
    if (ifGetSnapshot)
        QMessageBox::information(nullptr, tr("Info"), tr("Save snapshot to ") + ssname);
    else
        QMessageBox::warning(nullptr, tr("Warning"), tr("There are nothing to be taken! Please open a video or camera."));
}

QString SnapshotThread::getSnapshotName()
{
    return ssname;
}
