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
    try
    {
        start();
    }
    catch (const std::exception& ex)
    {
//        qDebug() << "Time out";
        QMessageBox::question(NULL, "question", "Content", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    }
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
//    QMessageBox::question(NULL, "question", "Content", QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
    qDebug() << "line 89";
    int win_w = vtkDisplay->width();
    int win_h = vtkDisplay->height();
    auto cam = ifm3d::Camera::MakeShared();
    auto fg = std::make_shared<ifm3d::FrameGrabber>(cam, 0xFFFF);
    auto buff = std::make_shared<ifm3d::ImageBuffer>();

    //
    // setup for point cloud
    //
    auto pclvis_ =
        std::make_shared<pcl::visualization::PCLVisualizer>("ifm3d-pcl-viewer");
    pclvis_->setSize(win_w, win_h);
    vtkDisplay->SetRenderWindow(pclvis_->getRenderWindow());
    int v_pcl(0);
    pclvis_->createViewPort(0., 0., 1., 1., v_pcl);
    pclvis_->setBackgroundColor(0, 0, 0, v_pcl);
    pclvis_->setCameraPosition(-3.0, // x-position
        0,    // y-position
        0,    // z-position
        0,    // x-axis "up" (0 = false)
        0,    // y-axis "up" (0 = false)
        1,    // z-axis "up" (1 = true)
        v_pcl);    // viewport

                   // use "A" and "a" to toggle axes indicators
    pclvis_->registerKeyboardCallback(
        [&](const pcl::visualization::KeyboardEvent& ev)
    {
        if (ev.getKeySym() == "A" && ev.keyDown())
        {
            pclvis_->addCoordinateSystem(1., "ifm3d", v_pcl);
        }
        else if (ev.getKeySym() == "a" && ev.keyDown())
        {
            pclvis_->removeCoordinateSystem("ifm3d");
        }
    });

    bool is_first = true;
    while (!pclvis_->wasStopped())
    {
        pclvis_->spinOnce(100);

        if (!fg->WaitForFrame(buff.get(), 500))
        {
            continue;
        }

        //------------
        // Point cloud
        //------------
        pcl::visualization::PointCloudColorHandlerGenericField<ifm3d::PointT>
            color_handler(buff->Cloud(), "intensity");

        if (is_first)
        {
            is_first = false;
            pclvis_->addPointCloud(buff->Cloud(), color_handler, "cloud",
                v_pcl);
//            vtkDisplay->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "pcl-viewer");
        }
        else
        {
            pclvis_->updatePointCloud(buff->Cloud(), color_handler, "cloud");
            vtkDisplay->update();
        }
    } // end: while (...)
}

IFM3DViewer::~IFM3DViewer()
{

}
