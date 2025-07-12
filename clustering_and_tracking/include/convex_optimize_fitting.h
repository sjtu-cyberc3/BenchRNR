
#ifndef CONVEX_OPTIMIZE_FITTING_H
#define CONVEX_OPTIMIZE_FITTING_H

#include <ros/ros.h>
#include <cmath>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <std_msgs/Float32.h>
#include <cmath>
#include <algorithm>

#include <std_msgs/String.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define __APP_NAME__ "Convex Optimize Fitting:"

static std_msgs::Float32 time_spent;
static double exe_time = 0.0;


class ConvexOptimizeFitting
{
private:
  int index_decrease(const int &index, const int &size);

  double Cal_VectorAngle(Eigen::Vector2f &line_vector, Eigen::Vector2f &lineseg);

  void eval_running_time(int running_time);

//  void eval_performance(double &theta_kitti, double &theta_optim, const uint32_t &index, const uint32_t &index_seq);

  void calcuBoxbyPolygon(double &theta_star, Eigen::Vector3d &position, Eigen::Vector3d &size, double &yaw,
                                             const std::vector<cv::Point2f> &polygon_cluster, const pcl::PointCloud<pcl::PointXYZI> &cluster);

  void optim_convex_fitting_improve(const std::vector<cv::Point2f> &ch_cluster, int &vl_idx_l, int &vl_idx_r, double &theta_optim, int &projL_idex_l_o, int &projL_idex_r_o, std::vector<cv::Point2f> &rcr_pts_o, pcl::PointCloud<pcl::PointXYZI>::Ptr &pt_test_visual);

public:
//  ConvexOptimizeFitting();
  void bbox_fitting(pcl::PointCloud<pcl::PointXYZI> current_cluster, Eigen::Vector3d &position, Eigen::Vector3d &size, double &yaw);
};

#endif  // CONVEX_OPTIMIZE_FITTING_H