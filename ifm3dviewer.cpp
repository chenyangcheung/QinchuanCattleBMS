#include "ifm3dviewer.h"
#include <QFileDialog>
#include <vtkRenderWindow.h>
#include <QDebug>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <ifm3d/camera.h>
#include <ifm3d/fg.h>
#include <ifm3d/image.h>
#include <QMessageBox>

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
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));
//    qDebug() << "line 19";
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "cloud");

    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    vtkDisplay->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkDisplay->GetInteractor(), vtkDisplay->GetRenderWindow());
    vtkDisplay->update();
    viewer->resetCamera();
}

void IFM3DViewer::openCamera(QString ifm3d_ip)
{
    IFM3D_IP = ifm3d_ip;

    cam = std::make_shared<ifm3d::Camera>(IFM3D_IP.toStdString());
    try
    {
        start();
    }
    catch (const std::exception& ex)
    {
        qDebug() << "Time out";
        return;
    }
}

void IFM3DViewer::openLocal()
{
//    vtkDisplay->SetRenderWindow(viewer->getRenderWindow());
//    viewer->setupInteractor(vtkDisplay->GetInteractor(), vtkDisplay->GetRenderWindow());

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
    try
    {
        auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 0xFFFF);
        auto buff = std::make_shared<ifm3d::ImageBuffer>();
        qDebug() << "Run to " << __LINE__;

        while (true)
        {
        //    viewer->spinOnce(100);

            if (!fg->WaitForFrame(buff.get(), 500))
            {
                break;
            }
            cloud = buff->Cloud();
            viewer->updatePointCloud<pcl::PointXYZI>(cloud, "cloud");
    //        viewer->resetCamera();
            vtkDisplay->update();
        } // end: while (...)
    }
    catch (const std::exception& ex)
    {
        qDebug() << ex.what();
        return;
    }
}

IFM3DViewer::~IFM3DViewer()
{

}
