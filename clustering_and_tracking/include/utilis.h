//
// Created by runrunxin on 24-3-19.
// 放一些小功能函数

#ifndef SRC_UTILIS_H
#define SRC_UTILIS_H

#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>

#include <unordered_map>

#include <iostream>


using namespace std;


class BoundingBox {
public:
    Eigen::Vector3f position;
    Eigen::Vector3f size;
    double theta;

    BoundingBox() {}

    BoundingBox(Eigen::Vector3f position, Eigen::Vector3f size, double theta) {
        this->position = position;
        this->size = size;
        this->theta = theta;
    }
};

// 点云下采样
using Voxel = Eigen::Vector3i;
struct VoxelHash
{
    size_t operator()(const Voxel &voxel) const
    {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};

// 降采样：随机选取一个体素内的点作为代表
pcl::PointCloud<pcl::PointXYZI> VoxelDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr frame, double voxel_size);

// 对当前帧和上一帧的点云总和进行降采样，并返回属于当前帧的、降采样后的点云
pcl::PointCloud<pcl::PointXYZI> VoxelDownsample_super(const pcl::PointCloud<pcl::PointXYZI>::Ptr frame, pcl::PointCloud<pcl::PointXYZI>::Ptr current_frame_downsampled,
                                                      int my_frame_num, double voxel_size);

Eigen::Vector2d calculateMarkerOrientation(const visualization_msgs::Marker& marker);

bool dbscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, std::vector<std::vector<int>> &clusters_index, const double &r, const int &size);
Eigen::MatrixXd pca(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in);
// void MyBoundingBox(pcl::PointCloud<pcl::PointXYZI> &class_point_cloud, pcl::PointXYZI &center_point, Eigen::MatrixXd &obj_pca, visualization_msgs::MarkerArray &marker_array, int &marker_id, double &length, double &width, double &height, ros::Time stamp_time, pcl::PointXYZI &point_zhou, int color_id, double bias = 1.3845, int flag = 0);
// void MyBoundingBox(pcl::PointCloud<pcl::PointXYZI> &class_point_cloud, pcl::PointXYZI &center_point, Eigen::MatrixXd &obj_pca, visualization_msgs::MarkerArray &marker_array, int &marker_id, double &length, double &width, double &height, ros::Time stamp_time, pcl::PointXYZI &point_zhou, int color_id, double bias = 1.3845);
void MyBoundingBox(pcl::PointCloud<pcl::PointXYZI> &class_point_cloud, pcl::PointXYZI &center_point, Eigen::MatrixXd &obj_pca, visualization_msgs::MarkerArray &marker_array, int &marker_id, double &length, double &width, double &height, ros::Time stamp_time, pcl::PointXYZI &point_zhou, int color_id, double bias = 1.3845, int flag = 0);

// void drawGTBBox(visualization_msgs::MarkerArray &marker_array, double timestamp);
void drawGTBBox(visualization_msgs::MarkerArray &marker_array, double timestamp, BoundingBox &gtbbox, std::string traj_num_str = "1");

void DrawBasedOnbbox_linebox(Eigen::Vector3f pos, double theta, Eigen::Vector3f size, visualization_msgs::MarkerArray &marker_array, int color_id = 0);

void simplebbox(pcl::PointCloud<pcl::PointXYZI> &class_point_cloud, Eigen::Vector3f &center_point, double &yaw, double &length, double &width, double &height); // 简单的包围盒计算函数

geometry_msgs::Point calculateMarkerCenter(const visualization_msgs::Marker& marker);

visualization_msgs::Marker DrawBasedOnbbox(Eigen::Vector3f pos, double theta, Eigen::Vector3f size, int marker_id, int color_id = 0);


pcl::PointCloud<pcl::PointXYZI> extractPointsInsideBoundingBox(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
    const BoundingBox& bbox);

void getEntryVertices(const pcl::PointXYZI &entry_center, const double &entry_length, const double &entry_width,
                      const double &entry_rot_angle, std::vector<pcl::PointXYZI> &vertices);

bool isPointInRegion(const pcl::PointXYZI &point, std::vector<pcl::PointXYZI> vertices);

bool isPointCloudInRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,std::vector<pcl::PointXYZI> vertices);

void create_entry_box(visualization_msgs::MarkerArray &marker_array_entry, pcl::PointXYZI &entry_center,
                      double &entry_length,double &entry_width, double &entry_rot_angle, ros::Time stamp_time);           

bool getTrajectoryParams(const std::string& filepath, int traj_num,
                        double& start_time, double& end_time, int& model_init, double& start_x, double& start_y);
#endif //SRC_UTILIS_H
