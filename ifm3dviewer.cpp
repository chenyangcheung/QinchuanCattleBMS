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
#include <pcl/io/pcd_io.h>
#include <ifm3d/image.h>
#include <QMessageBox>
#include <QDateTime>
#include <QApplication>

IFM3DViewer::IFM3DViewer(QObject *parent)
    : QThread(parent)
{
    camIsActive = false;
}

void IFM3DViewer::initViewer(QVTKWidget *&vd)
{
    vtkDisplay = vd;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
    intensity_distribution(cloud, "intensity");
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "cloud");

    viewer->addCoordinateSystem();
    int v_pcl(0);
//    viewer->createViewPort(0., 0., 1., 1., v_pcl);
//    viewer->setBackgroundColor(0, 0, 0, v_pcl);
    viewer->setCameraPosition(-3.0, // x-position
                              0,    // y-position
                              0,    // z-position
                              0,    // x-axis "up" (0 = false)
                              0,    // y-axis "up" (0 = false)
                              1,    // z-axis "up" (1 = true)
                              v_pcl);    // viewport

    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    vtkDisplay->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkDisplay->GetInteractor(), vtkDisplay->GetRenderWindow());
    vtkDisplay->update();
    viewer->resetCamera();
}

void IFM3DViewer::openCamera(QString ifm3d_ip)
{
    IFM3D_IP = ifm3d_ip;

    try
    {
        cam = std::make_shared<ifm3d::Camera>(IFM3D_IP.toStdString());
        fg = std::make_shared<ifm3d::FrameGrabber>(cam, 0xFFFF);
        camIsActive = true;
        start();
    }
    catch (const std::exception& ex)
    {
        camIsActive = false;
        QString qExcept;
//        qDebug() << ex.what();
        QMessageBox::warning(nullptr, tr("Warning"), qExcept.fromStdString(ex.what()));
        return;
    }
}

void IFM3DViewer::openLocal()
{
    if (camIsActive)
        closeCamera();

    filename = QFileDialog::getOpenFileName(nullptr,
               tr("Open PointCloud"), ".", tr("Open PCD files (*.pcd)"));
    if (!filename.isEmpty())
    {
        std::string file_name = filename.toStdString();
        pcl::PCLPointCloud2 cloud2;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        int pcd_version;
        int data_type;
        unsigned int data_idx;
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
        auto buff = std::make_shared<ifm3d::ImageBuffer>();
        while (camIsActive)
        {
            if (!fg->WaitForFrame(buff.get(), 500))
            {
                break;
            }
            buff->Cloud();
            cloud = buff->Cloud();
            viewer->updatePointCloud<pcl::PointXYZI>(cloud, "cloud");
            vtkDisplay->update();
        } // end: while (...)
        camIsActive = false;
    }
    catch (const std::exception& ex)
    {
        qDebug() << ex.what();
        return;
    }
    camIsActive = false;
}

void IFM3DViewer::takeSnapshot()
{
    if (cloud->empty())
    {
        QMessageBox::warning(nullptr, tr("Warning"), tr("There are nothing to be taken! Please open a pcd or camera."));
        return;
    }

    pcl::PointCloud<pcl::PointXYZI> *temp = cloud.get();

    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    ssname = qApp->applicationDirPath() + "/" + current_date + ".pcd";

    // Save pcd file by pcl library
//    pcl::io::savePCDFileASCII(ssname.toStdString(), *temp);
    pcl::PCDWriter writer;
    writer.write(ssname.toStdString(), *temp);

    QMessageBox::information(nullptr, tr("Info"), tr("Save snapshot to ") + ssname);
}

QString IFM3DViewer::getSnapshotName()
{
    return ssname;
}

void IFM3DViewer::stop()
{
    camIsActive = false;
}

void IFM3DViewer::closeCamera()
{
    stop();
}

IFM3DViewer::~IFM3DViewer()
{

}
