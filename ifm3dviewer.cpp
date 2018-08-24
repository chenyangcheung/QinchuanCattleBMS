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
#include <QFileInfo>
#include <QMessageBox>

IFM3DViewer::IFM3DViewer(QObject *parent)
    : QThread(parent)
{
    camIsActive = false;
    res = 0;
    isLocalPCD = false;
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
    viewer->setBackgroundColor (0.1, 0.1, 0.1);
    viewer->addCoordinateSystem (1.0, "cloud");
    vtkDisplay->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkDisplay->GetInteractor(), vtkDisplay->GetRenderWindow());
    vtkDisplay->update();
    viewer->resetCamera();
}

void IFM3DViewer::openCamera(QString ifm3d_ip)
{
    isLocalPCD = false;
    O3D3XX_IP = ifm3d_ip.toStdString();
    std::string PCIC_PORT_NUMBER = "80";
    std::string XMLRPC_PORT_NUMBER = "50010";
    SOURCE_PARAM = O3D3XX_IP + ":" + PCIC_PORT_NUMBER + ":" + XMLRPC_PORT_NUMBER;
    char err[256];
    camIsActive = true;

    res = pmdOpen(&hnd, SOURCE_PLUGIN, SOURCE_PARAM.c_str(), PROC_PLUGIN, PROC_PARAM);

    if (res != PMD_OK)
    {
        camIsActive = false;
        pmdGetLastError(hnd, err, 128);
        QMessageBox::warning(nullptr, "Warning", "Connect failed: " + QString::fromStdString(err));
        return;
    }

    // start o3d3xx camera thread
    start();

    // wait o3d3xx camera thread to quit
    quit();
    wait();

    // close o3d3xx camera
    res = pmdClose(hnd);
    if (res != PMD_OK)
    {
        camIsActive = false;
        pmdGetLastError(hnd, err, 128);
        QMessageBox::warning(nullptr, "Warning", "Could not close the connection " + QString::fromStdString(err));
        return;
    }
}

void IFM3DViewer::openLocal()
{
    isLocalPCD = true;
    if (camIsActive)
        closeCamera();

    filename = QFileDialog::getOpenFileName(nullptr,
               tr("Open PointCloud"), QDir::homePath(), tr("Open PCD files (*.pcd)"));
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
        viewer->resetCamera();
        vtkDisplay->update();
    }
}

void IFM3DViewer::run()
{
    int imgWidth;
    int imgHeight;
    res = pmdUpdate(hnd); // to update the camera parameter and framedata
    if (res != PMD_OK)
    {
        pmdGetLastError(hnd, pmd_errs, 256);
        camIsActive = false;
//        pmdClose(hnd);
        return;
    }

    res = pmdGetSourceDataDescription(hnd, &dd);
    if (res != PMD_OK)
    {
        pmdGetLastError(hnd, pmd_errs, 128);
        camIsActive = false;
//        pmdClose(hnd);
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
        res = pmdUpdate(hnd);
        if (res != PMD_OK)
        {
            camIsActive = false;
            break;
        }

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

        lock.unlock();
        viewer->updatePointCloud<pcl::PointXYZ>(cloud, "cloud");
        vtkDisplay->update();
    } // end: while (...)
}

void IFM3DViewer::takeSnapshot()
{
    if (cloud->empty())
    {
        QMessageBox::warning(nullptr, tr("Warning"), tr("There are nothing to be taken! Please open a pcd or camera."));
        return;
    }

    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    ssname = QFileInfo(snapshotPath).filePath() + "/" + current_date + ".pcd";

    // create temporary cloud to save
    pcl::PointCloud<pcl::PointXYZ> *temp(new pcl::PointCloud<pcl::PointXYZ>);
    if (!isLocalPCD)
    {
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
    }
    else
    {
        temp = cloud.get();
    }

    // Save pcd file by pcl library
    pcl::io::savePCDFile(ssname.toStdString(), *temp);

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

void IFM3DViewer::setSnapshotPath(QString ssp)
{
    snapshotPath = ssp;
}
