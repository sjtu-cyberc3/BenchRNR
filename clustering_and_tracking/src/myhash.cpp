#include "../include/myhash.hpp"

MyHash::MyHash(const Eigen::Vector3d& center,const Eigen::Vector3d& direction) {
    center_ = center;
    direction_ = direction;
    base_<<-direction_(1),direction_(0),0;
}

int MyHash::calHashKeyPolar(const pcl::PointXYZI& point) {
    // 把输入点投影到车辆坐标系
    double new_x,new_y,new_z;
    new_x = point.x;
    new_y = point.y;
    new_z = point.z;

    double theta = atan2(new_y,new_x) / M_PI * 180;
    double length = sqrt(new_x * new_x + new_y * new_y);
    int theta_index = floor((theta + 180) / ANGEL_RESOLUTION);
    int z_index = floor((new_z + 0.5*Z_RANGE) / RESOLUTION_POLAR);
    int length_index = floor((length + 0.5*LENGTH_RANGE)/ LENGTH_RESOLUTION);

    return z_index * floor(360 / ANGEL_RESOLUTION) + theta_index;
    // return z_index * floor(360 / ANGEL_RESOLUTION)*floor(LENGTH_RANGE/ LENGTH_RESOLUTION) + length_index * floor(360 / ANGEL_RESOLUTION) + theta_index;
}

int MyHash::calHashKey(const pcl::PointXYZI& point) {
    double new_x,new_y,new_z;
    new_x = point.x;
    new_y = point.y;
    new_z = point.z;

    int x_index = floor((new_x - (-0.5 * X_RANGE)) / RESOLUTION_GRID);
    int y_index = floor((new_y - (-0.5 * Y_RANGE)) / RESOLUTION_GRID);
    int z_index = floor((new_z - (-0.5 * Z_RANGE)) / RESOLUTION_GRID);

    int Key = z_index * (X_RANGE/RESOLUTION_GRID) * (Y_RANGE/RESOLUTION_GRID) + y_index * (X_RANGE/RESOLUTION_GRID) + x_index;
    return Key;
}

void MyHash::calHashMean() {
    // 先算出来在原坐标下的均值，再投影到主方向上
    for(auto it = hash_all.begin(); it != hash_all.end(); it++) {
        double mean_x = 0.0;
        double mean_y = 0.0;
        for(int i = 0; i <it->second.size(); i++) {
            mean_x += it->second[i].x;
            mean_y += it->second[i].y;
        }
        mean_x /= it->second.size();
        mean_y /= it->second.size();

        hash_mean_XY[it->first].first = mean_x;
        hash_mean_XY[it->first].second = mean_y;
    }
}


void MyHash::calHashCov() {
    for(auto it = hash_all.begin(); it != hash_all.end(); it++) {
        double cov_x = 0.0;
        double cov_y = 0.0;
        double delta_x = 0.0;
        double delta_y = 0.0;
        for(int i = 0; i <it->second.size(); i++) {
            delta_x = it->second[i].x;
            delta_y = it->second[i].y;
            cov_x += (delta_x - hash_mean_XY[it->first].first) * (delta_x - hash_mean_XY[it->first].first);
            cov_y += (delta_y - hash_mean_XY[it->first].second) * (delta_y - hash_mean_XY[it->first].second);
        }
        hash_cov_XY[it->first].first = cov_x / it->second.size();
        hash_cov_XY[it->first].second = cov_y / it->second.size();
    }
}