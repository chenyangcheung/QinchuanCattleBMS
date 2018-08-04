#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
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

class QWheelEvent;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void wheelEvent(QWheelEvent *event);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    VlcInstance *_instance;
    VlcMedia *_media;
    VlcMediaPlayer *_player;
    SnapshotThread SnapshotThread;
    IFM3DViewer ifm3dViewer;
    QGraphicsScene *imageScene;
    unsigned int dataCount;
    QString image2DName;    QString image3DName;
private slots:
    // 2d camera
    void openLocal();
    void openUrl();
    void takeSnapShot();
    // 2d image
    void add2DImage2Table();
    void removeImage();
    void display2dImage();
    // 3d camera
    void open3dCamera();
    void takeSnapShot3d();
    void adjustImageTableSize();
    // Compute BM
    void addData2Table();
};

#endif // MAINWINDOW_H
