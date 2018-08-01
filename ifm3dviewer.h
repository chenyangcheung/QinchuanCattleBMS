#ifndef IFM3DVIEWER_H
#define IFM3DVIEWER_H
#include <QVTKWidget.h>
#include <QString>
#include <QThread>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ifm3d/camera.h>

class IFM3DViewer : public QThread
{
    Q_OBJECT
public:
    IFM3DViewer(QObject *parent = 0);
    void initViewer(QVTKWidget *&vd);
    void openLocal();
    void openCamera(QString ifm3d_ip);
    ~IFM3DViewer();
protected:
    void virtual run() override;
private:
    QVTKWidget *vtkDisplay;
    QString filename;
    QString IFM3D_IP;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> realTimeViewer;
    ifm3d::Camera::Ptr cam;
};

#endif // IFM3DVIEWER_H
