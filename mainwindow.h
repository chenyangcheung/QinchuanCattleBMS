#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "snapshotthread.h"

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
//    VlcVideo *_video;
    SnapshotThread SnapshotThread;
private slots:
//    void openLocal();
//    void openUrl();
    void on_OpenFile_clicked();
    void on_OpenCamera_clicked();
    void TakeSnapShot();
};

#endif // MAINWINDOW_H
