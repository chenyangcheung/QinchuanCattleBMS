#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QGraphicsScene>
#include <QMainWindow>
#include <QButtonGroup>
#include <QVector>
#include <QCheckBox>
#include <QPair>

#include "snapshotthread.h"
#include "ifm3dviewer.h"
#include "imgmarkscene.h"
#include "imgpoint.h"
#include "bmscore.h"

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
    SnapshotThread vlc2DcameraSSThd;
    IFM3DViewer ifm3dViewer;
    QGraphicsScene *imageScene;
    ImgMarkScene *imgMarkScene;
    QButtonGroup *ptRatioBtnGroup;
    QButtonGroup *ptCheckboxGroup;
    unsigned int dataCount;
    QString image2DName;    QString image3DName;
    QPoint curPos;
//    QVector<QCheckBox*> ptCheckboxList;
    QVector<ImgPoint> pointList;
    BMScore bmscore;
    // flags
    bool ifComputed;      // check it before save BMI result, and it should be set true after trigger compute button
                          // and unset when triggering reset button
    bool useDefalutValue;
    bool ifConnect2DCam, ifConnect3DCam;
    QString snapshotPath;
    QVector<QPair<QString, QString>> computeGroupList;
    QString getFileNamePrefix();

    // File Tab
    VlcInstance *filesTabVlcInstance;
    VlcMedia *filesTabVlcMedia;
    VlcMediaPlayer *filesTabVlcPlayer;
    SnapshotThread filesTabVlcSSThd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr fileTabCloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> fileTab3DViewer;
    QVector<QString> files2DList;
    QVector<QString> files3DList;
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
    void toggleSelectedFlag();
    void updatePointInfo(qreal x, qreal y);
//    void addMark2Img(qreal x, qreal y);
    void savePointInfo(bool checked);
    void unsavePoint1Info(bool checked);
    void unsavePoint2Info(bool checked);
    void unsavePoint3Info(bool checked);
    void unsavePoint4Info(bool checked);
    void unsavePoint5Info(bool checked);
    void unsavePoint6Info(bool checked);
    void unsavePoint7Info(bool checked);
    void unsavePoint8Info(bool checked);
    bool checkIfAllSaved();
    void clearAll();
    void computeBodyMeasurement();
    void setBMScoreThreshold();
    void showSelectPointsHelp();
    void saveBMIToFile();
    void showAboutInfo();
    void showQtAbout();
    void setSanpshotPath();
    void displayVlcError();
    // Files Tab
    void filesTabOpenFile();
    void filesTabDelete();
    void filesTabVideoSnapshot();
    void filesTabSelect();
    void disableFilesTabSSBtn(int index);
    void files2DTabDisplayImg();
    void files3DTabDisplayImg();
};

#endif // MAINWINDOW_H
