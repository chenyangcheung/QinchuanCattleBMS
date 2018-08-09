#include "snapshotthread.h"
#include <QDateTime>
#include <QApplication>
#include <QDebug>
#include <QMessageBox>

SnapshotThread::SnapshotThread(QObject *parent)
    : QThread(parent)
{
    setCalParams();
}

void SnapshotThread::setCalParams()
{
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
    // === Intrinsic ===
    cameraMatrix.at<double>(0, 0) = 1302.728840;
    cameraMatrix.at<double>(0, 2) = 921.523419;
    cameraMatrix.at<double>(1, 1) = 1302.604297;
    cameraMatrix.at<double>(1, 2) = 578.403235;

    // === Distortion ===
    distCoeffs.at<double>(0, 0) = -0.380356;
    distCoeffs.at<double>(1, 0) = 0.200639;
    distCoeffs.at<double>(2, 0) = 0.000581;
    distCoeffs.at<double>(3, 0) = 0.000784;
//    distCoeffs.at<double>(4, 0) = 0;
//    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
//        cameraMatrix.at<double>(0, 0) = 4.450537506243416e+02;
//        cameraMatrix.at<double>(0, 1) = 0.192095145445498;
//        cameraMatrix.at<double>(0, 2) = 3.271489590204837e+02;
//        cameraMatrix.at<double>(1, 1) = 4.473690628394497e+02;
//        cameraMatrix.at<double>(1, 2) = 2.442734958206504e+02;

////        Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
//        distCoeffs.at<double>(0, 0) = -0.320311439187776;
//        distCoeffs.at<double>(1, 0) = 0.117708464407889;
//        distCoeffs.at<double>(2, 0) = -0.00548954846049678;
//        distCoeffs.at<double>(3, 0) = 0.00141925006352090;
//        distCoeffs.at<double>(4, 0) = 0;
}

void SnapshotThread::run()
{
    ifGetSnapshot = _video->takeSnapshot(ssname);
    delete _video;
}

void SnapshotThread::takeSnapshot(VlcMediaPlayer *&player)
{
    // Create a video environment
    // and don't forget delete after using it
    _video = new VlcVideo(player);

    // Generate image name according to current time
    QDateTime current_date_time = QDateTime::currentDateTime();
    QString current_date = current_date_time.toString("yyyy-MM-dd-hhmmsszzz");
    ssname = qApp->applicationDirPath() + "/" + current_date + ".jpg";

    // start to take snapshot
    start();

    // wait snapshot thread to quit
    quit();
    wait();
    if (ifGetSnapshot)
    {
        // TODO: CALIBRATE IMAGE
        imgCalibrate(ssname);
        QMessageBox::information(nullptr, tr("Info"), tr("Save snapshot to ") + ssname);
    }
    else
        QMessageBox::warning(nullptr, tr("Warning"), tr("There are nothing to be taken! Please open a video or camera."));
}

QString SnapshotThread::getSnapshotName()
{
    return ssname;
}

void SnapshotThread::imgCalibrate(QString imgName)
{
//    std::cout << imgName.toStdString() << std::endl;
    cv::Mat img = cv::imread(imgName.toLocal8Bit().toStdString());
    cv::Mat calibratedImg;

    cv::Mat map1, map2;
    cv::Size imgSize = img.size();
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize, 1, imgSize, 0),
                                imgSize, CV_16SC2, map1, map2);

    cv::remap(img, calibratedImg, map1, map2, cv::INTER_LINEAR);

    cv::imwrite(imgName.toStdString(), calibratedImg);
}
