#ifndef CERES_ICP_H
#define CERES_ICP_H



/*
 * @Descripttion: 
 * @version: 
 * @Author: sueRimn
 * @Date: 2022-07-28 13:02:19
 * @LastEditors: sueRimn
 * @LastEditTime: 2023-03-06 21:27:07
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>  
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/time.h>   // 利用控制台计算时间
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ceres/ceres.h>
#include <Eigen/Eigen>

// template <typename T>
// static T NormalizeAngle(const T& angle_degrees){
//     if (angle_degrees > T(180.0))
//         return angle_degrees - T(360.0);
//     else if (angle_degrees < T(-180.0))
//         return angle_degrees + T(360.0);
//     else
//         return angle_degrees;
// };

// class AngleLocalParameterization {
//     public:

//         template <typename T>
//         bool operator()(const T* theta_radians, const T* delta_theta_radians, 
//                         T* theta_radians_plus_delta) const {
//             *theta_radians_plus_delta = 
//                 NormalizeAngle(*theta_radians + *delta_theta_radians);

//             return true;
//         }

//         static ceres::LocalParameterization* Create(){
//             return(new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>);
//         }
// };

struct PP2dErrorTerm
{
public:
    PP2dErrorTerm(double obs_src_x, double obs_src_y, double obs_target_x, double obs_target_y,
                  Eigen::Matrix2d sqrt_information, double weight = 1.0)
        :obs_src_x_(obs_src_x), obs_src_y_(obs_src_y), obs_target_x_(obs_target_x), obs_target_y_(obs_target_y),
        sqrt_information_(sqrt_information), weight_(weight){}

    template <typename T>
    bool operator()(const T* yaw, const T* t_param, T* residuals) const
    {
        const Eigen::Matrix<T, 2, 1> t(t_param[0], t_param[1]); //平移量變成三維
        
        Eigen::Matrix<T, 2, 2> R1;
        R1 << cos(*yaw), -sin(*yaw), sin(*yaw), cos(*yaw); //如果要變成三維則需要更改矩陣表達式

        const Eigen::Matrix<T, 2, 2> R = R1;

        const Eigen::Vector2d obs_src_t(obs_src_x_, obs_src_y_);
        const Eigen::Vector2d obs_target_t(obs_target_x_, obs_target_y_);

        Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals);

        residuals_map.template head<2>() = (R * obs_src_t + t - obs_target_t.cast<T>());

        residuals_map = residuals_map * weight_;

        residuals_map = sqrt_information_.template cast<T>() * residuals_map;

        return true;
    }

    static ceres::CostFunction* Create(double  obs_src_x, double obs_src_y, double obs_target_x, double obs_target_y,
                                       Eigen::Matrix2d sqrt_information, double weight = 1.0)
    {
        return (new ceres::AutoDiffCostFunction<PP2dErrorTerm, 2, 1, 2>(
                new PP2dErrorTerm(obs_src_x, obs_src_y, obs_target_x, obs_target_y, sqrt_information, weight)));
    }

    double obs_src_x_, obs_src_y_;
    double obs_target_x_, obs_target_y_;
    Eigen::Matrix2d sqrt_information_;
    double weight_;
};

void my_icp(pcl::PointCloud<pcl::PointXYZI>& source, pcl::PointCloud<pcl::PointXYZI>& target, Eigen::Matrix3d& tf_matrix, bool transform_template = 1);

#endif