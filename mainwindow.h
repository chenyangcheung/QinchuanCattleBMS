#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "snapshotthread.h"
#include "ifm3dviewer.h"

namespace Ui {
class MainWindow;
}

class VlcInstance;
class VlcMedia;
class VlcMediaPlayer;
class VlcVideo;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    VlcInstance *_instance;
    VlcMedia *_media;
    VlcMediaPlayer *_player;
    SnapshotThread SnapshotThread;
    IFM3DViewer ifm3dViewer;
private slots:
    void openLocal();
    void openUrl();
    void takeSnapShot();
    void open3dCamera();
    void takeSnapShot3d();
};

#endif // MAINWINDOW_H
