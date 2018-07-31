#include "ifm3dviewer.h"
#include <QFileDialog>
#include <vtkRenderWindow.h>
#include <QDebug>

IFM3DViewer::IFM3DViewer(QObject *parent)
    : QThread(parent)
{
//    vtkDisplay = vd;
}

void IFM3DViewer::initViewer(QVTKWidget *&vd)
{
    vtkDisplay = vd;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
    intensity_distribution(cloud, "intensity");
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
//    qDebug() << "line 19";
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "cloud");

    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    vtkDisplay->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vd->GetInteractor(), vd->GetRenderWindow());
    vtkDisplay->update();
}

void IFM3DViewer::openCamera()
{

}

void IFM3DViewer::openLocal()
{
    filename = QFileDialog::getOpenFileName(nullptr,
               tr("Open PointCloud"), ".", tr("Open PCD files (*.pcd)"));
    if (!filename.isEmpty())
    {
        std::string file_name = filename.toStdString();
        //sensor_msgs::PointCloud2 cloud2;
        pcl::PCLPointCloud2 cloud2;
        //pcl::PointCloud<Eigen::MatrixXf> cloud2;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int pcd_version;
        int data_type;
        unsigned int data_idx;
//        int offset = 0;
        pcl::PCDReader rd;
        rd.readHeader(file_name, cloud2, origin, orientation, pcd_version, data_type, data_idx);

        if (data_type == 0)
        {
            pcl::io::loadPCDFile(filename.toStdString(), *cloud);
        }
        else if (data_type == 2)
        {
            pcl::PCDReader reader;
            reader.read<pcl::PointXYZI>(filename.toStdString(), *cloud);
        }

        viewer->updatePointCloud<pcl::PointXYZI>(cloud, "cloud");
        viewer->resetCamera();
        vtkDisplay->update();
    }
}

void IFM3DViewer::run()
{

}

IFM3DViewer::~IFM3DViewer()
{

}
