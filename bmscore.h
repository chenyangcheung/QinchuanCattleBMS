#ifndef BMSCORE_H
#define BMSCORE_H
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <string>

struct PtPos
{
    int x, y;
};

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
    std::vector<PtPos> ptPosList;
    int K;
    int threshold;
    //保留的八个位置+四个用户不可见的位置
    pcl::PointXYZ world_points[7];
  //  pcl::PointXYZ invisible_points[3];
    std::vector<std::vector<double>> middle;
    std::vector<std::vector<double>> neican;
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
public:
    BMScore();
    bool readCloudData(std::string cloudName);
    void initBMScore();
    void computeBodyMeasurement();
    double getWithersHeight();
    double getChestDepth();
    double getBackHeight();
    double getWaistHeight();
    double getHipHeight();
    double getRumpLength();
    double getBodyLength();
    int getThreshold();
    void setThreshold(int t);
    void setPtPosList(std::vector<PtPos> &p);
    std::vector<std::vector<double>> matrix_multiply(std::vector<std::vector<double>> arrA, std::vector<std::vector<double>> arrB);
    std::vector<std::vector<double>> division(std::vector<std::vector<double>> right,double Zc);
};

#endif // BMSCORE_H
