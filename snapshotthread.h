#ifndef SNAPSHOTTHREAD_H
#define SNAPSHOTTHREAD_H
#include <QThread>
#include <VLCQtCore/Video.h>
#include <VLCQtCore/MediaPlayer.h>
#include <QString>
#include <opencv2/opencv.hpp>

class SnapshotThread : public QThread
{
    Q_OBJECT
public:
    SnapshotThread(QObject *parent = 0);
    void takeSnapshot(VlcMediaPlayer *&player);
    QString getSnapshotName();
    void setCalParams();
    void imgCalibrate(QString imgName);
    void setSnapshotPath(QString ssp);
protected:
    void virtual run() override;
private:
    VlcVideo *_video;
    QString ssname;
    bool ifGetSnapshot;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    QString snapshotPath;
public:
    bool isImg;
};

#endif // SNAPSHOTTHREAD_H
