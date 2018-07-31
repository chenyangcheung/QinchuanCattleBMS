#ifndef IFM3DVIEWER_H
#define IFM3DVIEWER_H
#include <QVTKWidget.h>
#include <QString>
#include <QThread>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

class IFM3DViewer : public QThread
{
    Q_OBJECT
public:
    IFM3DViewer(QObject *parent = 0);
    void initViewer(QVTKWidget *&vd);
    void openLocal();
    void openCamera();
    ~IFM3DViewer();
protected:
    void virtual run() override;
private:
    QVTKWidget *vtkDisplay;
    QString filename;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
//    pcl::PointCloud::Ptr cloud;
//    pcl::PointCloud<PointT> c;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> realTimeViewer;
};

#endif // IFM3DVIEWER_H
