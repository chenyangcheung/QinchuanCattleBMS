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
#include <pcl/common/transforms.h>

IFM3DViewer::IFM3DViewer(QObject *parent)
    : QThread(parent)
{
    camIsActive = false;
    res = 0;
}

void IFM3DViewer::initViewer(QVTKWidget *&vd)
{
    vtkDisplay = vd;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>
//    intensity_distribution(cloud, "intensity");
    viewer.reset(new pcl::visualization::PCLVisualizer ("viewer", false));
//    viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "cloud");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

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
//    int v_pcl(0);
//    viewer->createViewPort(0., 0., 1., 1., v_pcl);
//    viewer->setBackgroundColor(0, 0, 0, v_pcl);
//    viewer->setCameraPosition(-3.0, // x-position
//                              0,    // y-position
//                              0,    // z-position
//                              0,    // x-axis "up" (0 = false)
//                              0,    // y-axis "up" (0 = false)
//                              1,    // z-axis "up" (1 = true)
//                              v_pcl);    // viewport
    viewer->addCoordinateSystem (1.0, "cloud", 0);
    vtkDisplay->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkDisplay->GetInteractor(), vtkDisplay->GetRenderWindow());
    vtkDisplay->update();
    viewer->resetCamera();
}

void IFM3DViewer::openCamera(QString ifm3d_ip)
{
    O3D3XX_IP = ifm3d_ip.toStdString();
    std::string PCIC_PORT_NUMBER = "80";
    std::string XMLRPC_PORT_NUMBER = "50010";
    SOURCE_PARAM = O3D3XX_IP + ":" + PCIC_PORT_NUMBER + ":" + XMLRPC_PORT_NUMBER;

//    IFM3D_IP = ifm3d_ip;

    try
    {
//        cam = std::make_shared<ifm3d::Camera>(IFM3D_IP.toStdString());
//        fg = std::make_shared<ifm3d::FrameGrabber>(cam, 0xFFFF);

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
            reader.read<pcl::PointXYZ>(filename.toStdString(), *cloud);
        }

        viewer->updatePointCloud<pcl::PointXYZ>(cloud, "cloud");
//        std::cout << matrix << std::endl;
        viewer->resetCamera();
        vtkDisplay->update();
    }
}

void IFM3DViewer::run()
{
    int imgWidth;
    int imgHeight;
    char err[256] = { 0 };
    try
    {
//        auto buff = std::make_shared<ifm3d::ImageBuffer>();
        // connect to camera
        res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM.c_str(), PROC_PLUGIN, PROC_PARAM);

        if (res != PMD_OK)
        {
            fprintf(stderr, "Could not connect: \n");
            getchar();
            return;
        }

        res = pmdUpdate(hnd); // to update the camera parameter and framedata
        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 256);
            fprintf(stderr, "Could not updateData: \n%s\n", err);
            pmdClose(hnd);
            printf("Camera Connection Closed. \n");
            return;
        }

        res = pmdGetSourceDataDescription(hnd, &dd);
        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Could not get data description: \n%s\n", err);
            pmdClose(hnd);
            return;
        }

        imgWidth = dd.img.numColumns;
        imgHeight = dd.img.numRows;
        xyz3Dcoordinate.resize(dd.img.numColumns * dd.img.numRows * 3);
        // init cloud info
        cloud->width = imgHeight * imgWidth;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
        flags.resize(imgWidth * imgHeight);

        while (camIsActive)
        {
//            if (!fg->WaitForFrame(buff.get(), 500))
//            {
//                break;
//            }
            res = pmdUpdate(hnd);
            if (res != PMD_OK)
            {
                break;
            }
//            buff->Cloud();
//            cloud = buff->Cloud();

            // update cloud data
            res = pmdGetFlags(hnd, &flags[0], flags.size() * sizeof(float));
            res = pmdGet3DCoordinates(hnd, &xyz3Dcoordinate[0], xyz3Dcoordinate.size() * sizeof(float));

            lock.lockForWrite();

            for (size_t i = 0; i < imgHeight * imgWidth; i++)
            {
                cloud->points[i].x = xyz3Dcoordinate[i * 3 + 0];
                cloud->points[i].y = xyz3Dcoordinate[i * 3 + 1];
                cloud->points[i].z = xyz3Dcoordinate[i * 3 + 2];
            }

//            cloud->width = counter;
//            cloud->points.resize(counter);
            lock.unlock();
            viewer->updatePointCloud<pcl::PointXYZ>(cloud, "cloud");
            vtkDisplay->update();
        } // end: while (...)
        camIsActive = false;
    }
    catch (const std::exception& ex)
    {
        res = pmdClose(hnd);
        if (res != PMD_OK)
        {
            pmdGetLastError(hnd, err, 128);
            fprintf(stderr, "Could not close the connection %s\n", err);
        }
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

    // create temporary cloud to save
    pcl::PointCloud<pcl::PointXYZ> *temp(new pcl::PointCloud<pcl::PointXYZ>);
    temp->width = cloud->width;
    temp->height = cloud->height;
    temp->is_dense = false;
    temp->points.resize(temp->width * temp->height);

    // filter invalid points
    int counter = 0;
    for (size_t i = 0; i < temp->width; i++)
    {
        if (!(flags[i] & 1))
        {
            temp->points[i].x = xyz3Dcoordinate[i * 3 + 0];
            temp->points[i].y = xyz3Dcoordinate[i * 3 + 1];
            temp->points[i].z = xyz3Dcoordinate[i * 3 + 2];
            counter++;
        }
    }
    temp->width = counter;
    temp->points.resize(counter);

    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    ssname = qApp->applicationDirPath() + "/" + current_date + ".pcd";

    lock.lockForRead();
    // Save pcd file by pcl library
    pcl::io::savePCDFile(ssname.toStdString(), *temp);
    lock.unlock();

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
    res = pmdClose(hnd);
}

IFM3DViewer::~IFM3DViewer()
{

}
