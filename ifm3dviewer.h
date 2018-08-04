#ifndef IFM3DVIEWER_H
#define IFM3DVIEWER_H
#include <QVTKWidget.h>
#include <QString>
#include <QThread>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>

class IFM3DViewer : public QThread
{
    Q_OBJECT
public:
    IFM3DViewer(QObject *parent = 0);
    void initViewer(QVTKWidget *&vd);
    void openLocal();
    void openCamera(QString ifm3d_ip);
    void takeSnapshot();
    void closeCamera();
    void stop();
    QString getSnapshotName();
    ~IFM3DViewer();
protected:
    void virtual run() override;
private:
    QVTKWidget *vtkDisplay;
    QString filename;
    QString ssname;
    QString IFM3D_IP;
    bool camIsActive;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> realTimeViewer;
    ifm3d::Camera::Ptr cam;
    ifm3d::FrameGrabber::Ptr fg;
};

#endif // IFM3DVIEWER_H
