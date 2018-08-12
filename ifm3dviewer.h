#ifndef IFM3DVIEWER_H
#define IFM3DVIEWER_H
#include <QVTKWidget.h>
#include <QString>
#include <QThread>
#include <QReadWriteLock>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pmdsdk2.h>
//#include <ifm3d/camera.h>
//#include <ifm3d/fg.h>
// on error, prepend absolute path to files before plugin names
#define SOURCE_PLUGIN "O3D3xxCamera"

#define PROC_PLUGIN "O3D3xxProc"
#define PROC_PARAM ""

// Define length of buffer for response from source command
#define SOURCE_CMD_BUFFER_LENGTH 256

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
//    QString IFM3D_IP;
    std::string O3D3XX_IP;
    bool camIsActive;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> realTimeViewer;
//    ifm3d::Camera::Ptr cam;
//    ifm3d::FrameGrabber::Ptr fg;
    std::vector<float> xyz3Dcoordinate;
    std::vector<unsigned> flags;
    std::string SOURCE_PARAM;
    int res;
    PMDHandle hnd = 0; // connection handle
    PMDDataDescription dd;
    QReadWriteLock lock;
};

#endif // IFM3DVIEWER_H
