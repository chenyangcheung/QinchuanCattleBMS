#include "bmscore.h"

BMScore::BMScore()
{
    // set search point is origin point
    searchPoint.x = 0;
    searchPoint.y = 0;
    searchPoint.z = 0;
}

void BMScore::computeBodyMeasurement()
{
//    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0)
//        {
//            for (size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
//                //if(right-error<left<right+error)
//                //then cout & markup
//                double cloud_after_x = cloud->points[pointIdxNKNSearch[i]].x + 0.20247;
//                double cloud_after_y = cloud->points[pointIdxNKNSearch[i]].y - 0.00161;
//                double cloud_after_z = cloud->points[pointIdxNKNSearch[i]].z - 0.019;

//                vector<vector<double>> C = { { cloud_after_x },{ cloud_after_y },{ cloud_after_z } };

//                vector<vector<double>> middle = division(matrix_multiply(neican, C), cloud_after_z);

//                double u = middle[0][0];
//                double v = middle[1][0];


//                for(int j=0; j<8; j++){
//                if(u <= x[j] + threshold && u >= x[j] - threshold && v <= y[j] + threshold && v >= y[j] - threshold){
//                    world_points[j].x = cloud->points[pointIdxNKNSearch[i]].x;
//                    world_points[j].y = cloud->points[pointIdxNKNSearch[i]].y;
//                    world_points[j].z = cloud->points[pointIdxNKNSearch[i]].z;
//                }
//                for(int k=0; k<4; k++){
//                if(u <= xi[k] + threshold && u >= xi[k] - threshold && v <= yi[k] + threshold && v >= yi[k] - threshold){
//                    invisible_points[k].x = cloud->points[pointIdxNKNSearch[i]].x;
//                    invisible_points[k].y = cloud->points[pointIdxNKNSearch[i]].y;
//                    invisible_points[k].z = cloud->points[pointIdxNKNSearch[i]].z;
//                }
//                }
//            }
//        }
//        //计算各项体尺数值
//        withers_height = sqrt((world_points[0].x-word_points[1].x)*(world_points[0].x-word_points[1].x) + (world_points[0].y-world_points[1].y)*(world_points[0].y-world_points[1].y) + (world_points[0].z-world_points[1].z)*(world_points[0].z-world_points[1].z));
//        chest_depth =  sqrt((world_points[2].x-word_points[3].x)*(world_points[2].x-word_points[3].x) + (world_points[2].y-world_points[3].y)*(world_points[2].y-world_points[3].y) + (world_points[2].z-world_points[3].z)*(world_points[2].z-world_points[3].z));
//        back_height = sqrt((world_points[4].x-invisible_points[0].x)*(world_points[4].x-invisible_points[0].x) + (world_points[4].y-invisible_points[0].y)*(world_points[4].y-invisible_points[0].y) + (world_points[4].z-invisible_points[0].z)*(world_points[4].z-invisible_points[0].z));
//        waist_height = sqrt((world_points[5].x-invisible_points[1].x)*(world_points[5].x-invisible_points[1].x) + (world_points[5].y-invisible_points[1].y)*(world_points[5].y-invisible_points[1].y) + (world_points[5].z-invisible_points[1].z)*(world_points[5].z-invisible_points[1].z));
//        hip_height = sqrt((world_points[7].x-invisible_points[2].x)*(world_points[7].x-invisible_points[2].x) + (world_points[7].y-invisible_points[2].y)*(world_points[7].y-invisible_points[2].y) + (world_points[7].z-invisible_points[2].z)*(world_points[7].z-invisible_points[2].z));
//        rump_length = sqrt((world_points[7].x-invisible_points[3].x)*(world_points[7].x-invisible_points[3].x) + (world_points[7].y-invisible_points[3].y)*(world_points[7].y-invisible_points[3].y) + (world_points[7].z-invisible_points[3].z)*(world_points[7].z-invisible_points[3].z));
//        body_length = sqrt((world_points[6].x-word_points[7].x)*(world_points[6].x-word_points[7].x) + (world_points[6].y-world_points[7].y)*(world_points[6].y-world_points[7].y) + (world_points[6].z-world_points[7].z)*(world_points[6].z-world_points[7].z));
}
