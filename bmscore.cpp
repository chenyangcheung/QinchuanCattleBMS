#include "bmscore.h"
#include <QDebug>

BMScore::BMScore()
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    threshold = 10;
    for (int i = 0; i < 8; i++)
    {
        PtPos p;
        p.x = 0; p.y = 0;
        ptPosList.push_back(p);
    }

}

int BMScore::getThreshold()
{
    return threshold;
}

void BMScore::setThreshold(int t)
{
    threshold = t;
}

void BMScore::setPtPosList(std::vector<PtPos> &p)
{
    for (int i = 0; i < 8; i++)
    {
        ptPosList[i].x = p[i].x;
        ptPosList[i].y = p[i].y;
    }
}

void BMScore::initBMScore()
{
    // set search point is origin point
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;
    neican.push_back({ 1302.729 , 0 , 921.523 });
    neican.push_back({ 0 , 1302.604 , 578.403 });
    neican.push_back({ 0 , 0  ,1 });
}

bool BMScore::readCloudData(std::string cloudName)
{
    if (cloudName.empty())
        return false;
    reader.read(cloudName, *cloud);
    qDebug() << "Read cloud Success: " << __LINE__;
    return true;
}

std::vector<std::vector<double>> BMScore::division(std::vector<std::vector<double>> right,double Zc)
{
    //最后返回的数组,先随便分配成0,0,0,
    std::vector<std::vector<double>> after;
    after.resize(3);
    for (int i = 0; i < 3; ++i)
    {
        after[i].resize(1);
    }

    for (int i = 0;i < 3;i++) {
        after[i][0] = right[i][0] / Zc;
    }

    double middle = after[2][0];
    for (int i = 0;i < 3;i++) {
        after[i][0] = after[i][0] / middle;
    }

    return after;
}

std::vector<std::vector<double>> BMScore::matrix_multiply(std::vector<std::vector<double>> arrA, std::vector<std::vector<double>> arrB)
{
    //矩阵arrA的行数
    int rowA = arrA.size();
    int colA = arrA[0].size();
    //矩阵arrB的行数
    int rowB = arrB.size();
    //矩阵arrB的列数
    int colB = arrB[0].size();
    //相乘后的结果矩阵
    std::vector<std::vector<double>>  res;
    if (colA != rowB)//如果矩阵arrA的列数不等于矩阵arrB的行数。则返回空
    {
        return res;
    }
    else
    {
        //设置结果矩阵的大小，初始化为为0
        res.resize(rowA);
        for (int i = 0; i < rowA; ++i)
        {
            res[i].resize(colB);
        }

        //矩阵相乘
        for (int i = 0; i < rowA; ++i)
        {
            for (int j = 0; j < colB; ++j)
            {
                for (int k = 0; k < colA; ++k)
                {
                    res[i][j] += arrA[i][k] * arrB[k][j];
                    //cout<< setprecision(8) << res[i][j] << endl;
                }
            }
        }
    }
    return res;
}

void BMScore::computeBodyMeasurement()
{
    K = cloud->width * cloud->height;
    std::vector<int>pointIdxNKNSearch(K); //存储查询点近邻索引
    std::vector<float>pointNKNSquaredDistance(K); //存储近邻点对应距离平方
    int x[8], y[8];
    int xi[4], yi[4];

    qDebug() << "Get Point List: ";
    for (int i = 0; i < 8; i++)
    {
        x[i] = ptPosList[i].x;
        y[i] = ptPosList[i].y;
        qDebug() << "P" << i << ":" << "(" << x[i] << "," << y[i] << ")";
    }

    kdtree.setInputCloud(cloud); //设置搜索空间
    //Back_height B
    xi[0] = x[4];
    yi[0] = y[1];
    //Waist_height B
    xi[1] = x[5];
    yi[1] = y[1];
    //Hip_height B
    xi[2] = x[7];
    yi[2] = y[1];
    //Rump_length A
    xi[3] = x[5];
    yi[3] = y[7];

    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        qDebug() << "Running to line: " << __LINE__;
        for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
        {
            //if(right-error<left<right+error)
            //then cout & markup
            double cloud_after_x = cloud->points[pointIdxNKNSearch[i]].x + 0.20247;
            double cloud_after_y = cloud->points[pointIdxNKNSearch[i]].y - 0.00161;
            double cloud_after_z = cloud->points[pointIdxNKNSearch[i]].z - 0.019;

            std::vector<std::vector<double>> C = { { cloud_after_x },{ cloud_after_y },{ cloud_after_z } };

            std::vector<std::vector<double>> middle = division(matrix_multiply(neican, C), cloud_after_z);

            double u = middle[0][0];
            double v = middle[1][0];

            for(int j=0; j<8; j++)
            {
                if(u <= x[j] + threshold && u >= x[j] - threshold && v <= y[j] + threshold && v >= y[j] - threshold)
                {
                    world_points[j].x = cloud->points[pointIdxNKNSearch[i]].x;
                    world_points[j].y = cloud->points[pointIdxNKNSearch[i]].y;
                    world_points[j].z = cloud->points[pointIdxNKNSearch[i]].z;
                    qDebug() << "World Point: " << "(" << world_points[j].x << "," << world_points[j].y << "," << world_points[j].z << ")";
                }
            }
            for(int k=0; k<4; k++)
            {
                if(u <= xi[k] + threshold && u >= xi[k] - threshold && v <= yi[k] + threshold && v >= yi[k] - threshold)
                {
                    invisible_points[k].x = cloud->points[pointIdxNKNSearch[i]].x;
                    invisible_points[k].y = cloud->points[pointIdxNKNSearch[i]].y;
                    invisible_points[k].z = cloud->points[pointIdxNKNSearch[i]].z;
                    qDebug() << "Invisible Point: " << "(" << invisible_points[k].x << "," << invisible_points[k].y << "," << invisible_points[k].z << ")";
                }
            }
        }
    }
    //计算各项体尺数值
    withers_height = sqrt((world_points[0].x-world_points[1].x)*(world_points[0].x-world_points[1].x) + (world_points[0].y-world_points[1].y)*(world_points[0].y-world_points[1].y) + (world_points[0].z-world_points[1].z)*(world_points[0].z-world_points[1].z));
    chest_depth =  sqrt((world_points[2].x-world_points[3].x)*(world_points[2].x-world_points[3].x) + (world_points[2].y-world_points[3].y)*(world_points[2].y-world_points[3].y) + (world_points[2].z-world_points[3].z)*(world_points[2].z-world_points[3].z));
    back_height = sqrt((world_points[4].x-invisible_points[0].x)*(world_points[4].x-invisible_points[0].x) + (world_points[4].y-invisible_points[0].y)*(world_points[4].y-invisible_points[0].y) + (world_points[4].z-invisible_points[0].z)*(world_points[4].z-invisible_points[0].z));
    waist_height = sqrt((world_points[5].x-invisible_points[1].x)*(world_points[5].x-invisible_points[1].x) + (world_points[5].y-invisible_points[1].y)*(world_points[5].y-invisible_points[1].y) + (world_points[5].z-invisible_points[1].z)*(world_points[5].z-invisible_points[1].z));
    hip_height = sqrt((world_points[7].x-invisible_points[2].x)*(world_points[7].x-invisible_points[2].x) + (world_points[7].y-invisible_points[2].y)*(world_points[7].y-invisible_points[2].y) + (world_points[7].z-invisible_points[2].z)*(world_points[7].z-invisible_points[2].z));
    rump_length = sqrt((world_points[7].x-invisible_points[3].x)*(world_points[7].x-invisible_points[3].x) + (world_points[7].y-invisible_points[3].y)*(world_points[7].y-invisible_points[3].y) + (world_points[7].z-invisible_points[3].z)*(world_points[7].z-invisible_points[3].z));
    body_length = sqrt((world_points[6].x-world_points[7].x)*(world_points[6].x-world_points[7].x) + (world_points[6].y-world_points[7].y)*(world_points[6].y-world_points[7].y) + (world_points[6].z-world_points[7].z)*(world_points[6].z-world_points[7].z));
}

double BMScore::getWithersHeight()
{
    return withers_height;
}

double BMScore::getChestDepth()
{
    return chest_depth;
}

double BMScore::getBackHeight()
{
    return back_height;
}

double BMScore::getWaistHeight()
{
    return waist_height;
}

double BMScore::getHipHeight()
{
    return hip_height;
}

double BMScore::getRumpLength()
{
    return rump_length;
}

double BMScore::getBodyLength()
{
    return body_length;
}
