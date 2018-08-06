#ifndef BMSCORE_H
#define BMSCORE_H
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <vector>

class BMScore
{
private:
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointXYZ searchPoint;
    double withers_height;
    double chest_depth;
    double back_height;
    double waist_height;
    double hip_height;
    double rump_length;
    double body_length;
    int K;
    //保留的八个位置+四个用户不可见的位置
    pcl::PointXYZ world_points[7];
    pcl::PointXYZ invisible_points[3];
    std::vector<std::vector<double>> middle;
    std::vector<std::vector<double>> neican;
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
public:
    BMScore();
    void computeBodyMeasurement();
    double getWithersHeight();
    double getChestDepth();
    double getBackHeight();
    double getWaistHeight();
    double getHipHeight();
    double getRumpLength();
    double bodyLength();
};

#endif // BMSCORE_H
