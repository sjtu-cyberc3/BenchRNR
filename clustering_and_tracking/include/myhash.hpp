#ifndef _MYHASH_HPP_
#define _MYHASH_HPP_

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <unordered_map>

using namespace std;



#define X_RANGE 50
#define Y_RANGE 50
#define Z_RANGE 10 //划定栅格化的范围，从而得到固定的用于进行哈希化的索引，一般无需改动
#define LENGTH_RANGE 20
#define RESOLUTION_POLAR 0.2 //使用极坐标时的纵向栅格分辨率/m
#define ANGEL_RESOLUTION 20 //使用极坐标的水平角度分辨率/°
#define RESOLUTION_GRID 0.2 //使用空间栅格的栅格分辨率/m
#define LENGTH_RESOLUTION 0.2

class MyHash{
private:
    Eigen::Vector3d center_;
    Eigen::Vector3d direction_;
    Eigen::Vector3d base_;
public:
    unordered_map<int,pcl::PointCloud<pcl::PointXYZI>> hash_all; //用于存储全部点云的哈希表
    unordered_map<int,pair<double,double>> hash_mean_XY;             //用于存储所有栅格的均值
    unordered_map<int,pair<double,double>> hash_cov_XY;              //用于存储所有栅格的方差

    MyHash(const Eigen::Vector3d& center,const Eigen::Vector3d& direction);
    int calHashKeyPolar(const pcl::PointXYZI& point);  // 极坐标栅格划
    int calHashKey(const pcl::PointXYZI& point);       // X、Y坐标栅格划分
    void calHashMean();
    void calHashCov();
};

#endif