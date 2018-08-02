#ifndef SNAPSHOTTHREAD_H
#define SNAPSHOTTHREAD_H
#include <QThread>
#include <VLCQtCore/Video.h>
#include <VLCQtCore/MediaPlayer.h>
#include <QString>

class SnapshotThread : public QThread
{
    Q_OBJECT
public:
    SnapshotThread(QObject *parent = 0);
    void takeSnapshot(VlcMediaPlayer *&player);
protected:
    void virtual run() override;
private:
    VlcVideo *_video;
    QString ssname;
    bool ifGetSnapshot;
};

#endif // SNAPSHOTTHREAD_H
