#ifndef _MY_OPTIMIZER_HPP_
# define _MY_OPTIMIZER_HPP_



#include <ceres/ceres.h>
#include <pcl/point_types.h>




// 角度的局部参数化
// template<typename T>
// static T NormalizeAngle(const T& angle) {
//     if(angle > T(180.0)) {
//         return angle - T(360.0);
//     } else if(angle < T(-180.0)) {
//         return angle + T(360.0);
//     } else {
//         return angle;
//     }
// }

const float PI = 3.141592;


template<typename T>
static T NormalizeAngle(const T& angle) {
    if(angle > T(PI)) {
        return angle - T(2 * PI);
    } else if(angle < T(-PI)) {
        return angle + T(2 * PI);
    } else {
        return angle;
    }
}

// class AngleLocalParameterization {
//     public:
//     template<typename T>
//     bool operator() (const T* angle, const T* delta_angle, T* angle_plus_delta) const{
//         *angle_plus_delta = NormalizeAngle(*angle + *delta_angle);
//         return true;
//     }
//     static ceres::LocalParameterization* Create() {
//         return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,1,1>);
//     }
// };


class localOptimizer {
public:
    localOptimizer(const pcl::PointXYZI point,const Eigen::Vector3d & center, const Eigen::Vector3d& dir, const double& mean_x, const double& mean_y) {
        point_ = point;
        center_ = center;
        dir_ = dir;
        mean_x_ = mean_x;
        mean_y_ = mean_y;
    }

    template <typename T>
    bool operator() (const T* yaw, const T* t, T* residual) const {
        T mean_x = (T)mean_x_;
        T mean_y = (T)mean_y_;

        T tf_x = (T)point_.x * cos(*yaw) + (T)point_.y * sin(*yaw) + t[0];
        T tf_y = - (T)point_.x * sin(*yaw) + (T)point_.y * cos(*yaw) + t[1];


        T accum_x = (T)0.0;
        T accum_y = (T)0.0;

        accum_x = (tf_x - mean_x) * (tf_x -mean_x);
        accum_y = (tf_y - mean_y) * (tf_y -mean_y);

        residual[0] = ceres::sqrt(std::max(T(0.0), accum_x + accum_y));

        return 1;
    }

    static ceres::CostFunction * Create(const pcl::PointXYZI point,const Eigen::Vector3d & center, const Eigen::Vector3d& dir, const double& mean_x, const double& mean_y) {
        return(
            new ceres::AutoDiffCostFunction<localOptimizer,1,1,2>(
                new localOptimizer(point, center, dir, mean_x, mean_y)
            )
        );
    }
private:
    pcl::PointXYZI point_;
    Eigen::Vector3d center_;
    Eigen::Vector3d dir_;
    double mean_x_;
    double mean_y_;
};


class localOptimizer_x {
public:
    localOptimizer_x(const pcl::PointXYZI point,const Eigen::Vector3d & center, const Eigen::Vector3d& dir, const double& mean_x, const double& mean_y) {
        point_ = point;
        center_ = center;
        dir_ = dir;
        mean_x_ = mean_x;
        mean_y_ = mean_y;
    }

    template <typename T>
    bool operator() (const T* yaw, const T* t, T* residual) const {
        T mean_x = (T)mean_x_;
        T mean_y = (T)mean_y_;

        T tf_x = (T)point_.x * cos(*yaw) + (T)point_.y * sin(*yaw) + t[0];
        T tf_y = - (T)point_.x * sin(*yaw) + (T)point_.y * cos(*yaw) + t[1];


        T accum_x = (T)0.0;

        accum_x = (tf_x - mean_x) * (tf_x -mean_x);

        residual[0] = ceres::sqrt(accum_x);

        return 1;
    }

    static ceres::CostFunction * Create(const pcl::PointXYZI point,const Eigen::Vector3d & center, const Eigen::Vector3d& dir, const double& mean_x, const double& mean_y) {
        return(
            new ceres::AutoDiffCostFunction<localOptimizer_x,1,1,2>(
                new localOptimizer_x(point, center, dir, mean_x, mean_y)
            )
        );
    }
private:
    pcl::PointXYZI point_;
    Eigen::Vector3d center_;
    Eigen::Vector3d dir_;
    double mean_x_;
    double mean_y_;
};


class localOptimizer_y {
public:
    localOptimizer_y(const pcl::PointXYZI point,const Eigen::Vector3d & center, const Eigen::Vector3d& dir, const double& mean_x, const double& mean_y) {
        point_ = point;
        center_ = center;
        dir_ = dir;
        mean_x_ = mean_x;
        mean_y_ = mean_y;
    }

    template <typename T>
    bool operator() (const T* yaw, const T* t, T* residual) const {
        T mean_x = (T)mean_x_;
        T mean_y = (T)mean_y_;

        T tf_x = (T)point_.x * cos(*yaw) + (T)point_.y * sin(*yaw) + t[0];
        T tf_y = - (T)point_.x * sin(*yaw) + (T)point_.y * cos(*yaw) + t[1];


        T accum_y = (T)0.0;

        accum_y = (tf_y - mean_y) * (tf_y -mean_y);

        residual[0] = ceres::sqrt(accum_y);

        return 1;
    }

    static ceres::CostFunction * Create(const pcl::PointXYZI point,const Eigen::Vector3d & center, const Eigen::Vector3d& dir, const double& mean_x, const double& mean_y) {
        return(
            new ceres::AutoDiffCostFunction<localOptimizer_y,1,1,2>(
                new localOptimizer_y(point, center, dir, mean_x, mean_y)
            )
        );
    }
private:
    pcl::PointXYZI point_;
    Eigen::Vector3d center_;
    Eigen::Vector3d dir_;
    double mean_x_;
    double mean_y_;
};


# endif