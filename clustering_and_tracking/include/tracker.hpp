//
// Created by runrunxin on 24-3-19.
//

#ifndef SRC_TRACKER_HPP
#define SRC_TRACKER_HPP


#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <deque>
#include <unordered_set>



#include "../include/Hungarian.h"
#include "../include/ceres_icp.h"
#include "../include/utilis.h"
#include "../include/myhash.hpp"
#include "../include/myoptimizer.hpp"
#include "../include/convex_optimize_fitting.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <future>




class Tracker {
public:
    // tracker id
    int tracker_id = 0;
    int time_since_update = 0;

    int hit_count = 0;
    int life_span = 0;

    int static_time = 0;
    bool is_static = false;

    Eigen::Vector3f position_estimate = Eigen::Vector3f::Zero();
    Eigen::Vector3f size_estimate = Eigen::Vector3f::Zero();
    Eigen::Vector3f delta_pos_estimate = Eigen::Vector3f::Zero();
    double theta_estimate = 0;

    pcl::PointCloud<pcl::PointXYZI> point_cloud;

    // define a 3D Kalman Filter
    cv::KalmanFilter *kf;

    void init(int id);

    void predict(Eigen::Vector3f &position, Eigen::Vector3f &size, double &theta);
    void update(Eigen::Vector3f position, Eigen::Vector3f size, double theta);
};

class MOT3D {
public:
    MOT3D() {};
    ~MOT3D() {};

    int max_age = 20;
    int min_hits = 5;
    int id_count = 0;

    int max_static = 20;
    double vel_threshold = 0.01;
    double time_between_two_update = 0.0227; 

    std::vector<pcl::PointCloud<pcl::PointXYZI>> class_point_clouds_vector;

    // define Tracker vector
    std::vector<Tracker> trackers;
    int find_index_by_id(int id);

    void predict(std::vector<BoundingBox> &bbox_predicted);
    void update(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_predict);

    void birthAndDeath(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_observe, std::vector<int> assignment_predict);
};


// Target association
// Input: observed bounding boxes, predicted bounding boxes
// Output: 
//   assignment_predict — for each predicted bbox, the index of the matched observed bbox; 
//                        if a prediction is unmatched, then assignment_predict[i] = -1;
//   assignment_observe — for each observed bbox, the index of the matched predicted bbox; 
//                        if an observation j is unmatched, then assignment_observe[j] = -1;

void DataAssociation(std::vector<BoundingBox> bbox_observed, std::vector<BoundingBox> bbox_predicted, std::vector<int> &assignment_predict,std::vector<int> &assignment_observe, std::vector<double> &cost_vector);


float boxArea(const BoundingBox& box);

# endif