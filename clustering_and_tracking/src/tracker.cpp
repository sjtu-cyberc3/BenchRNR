//
// Created by runrunxin on 24-3-19.
//

#include "../include/tracker.hpp"


void Tracker::init(int id){
    tracker_id = id;

    int state_dim = 10;
    int measure_dim = 7;
    int control_dim = 0;

    kf = new cv::KalmanFilter(state_dim, measure_dim, control_dim);

    // initialize Kalman Filter

    // Define state transition matrix F
    // state x dimension 10: x, y, z, theta, l, w, h, dx, dy, dz
    // constant velocity model: x' = x + dx, y' = y + dy, z' = z + dz
    // while all others (theta, l, w, h, dx, dy, dz) remain the same
    // state transition matrix, dim_x * dim_x
    cv::setIdentity(kf->transitionMatrix);
    kf->transitionMatrix.at<float>(0, 7) = 0.1;
    kf->transitionMatrix.at<float>(1, 8) = 0.1;
    kf->transitionMatrix.at<float>(2, 9) = 0.1;

    // Define measurement matrix H
    // measurement matrix, dim_z * dim_x, the first 7 dimensions of the measurement correspond to the state
    kf->measurementMatrix = (cv::Mat_<float>(7, 10) <<
                                                    1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 0, 0, 0);

    // Define process noise covariance matrix Q
    cv::setIdentity(kf->processNoiseCov, cv::Scalar::all(0.1));

    // Define measurement noise covariance matrix R
    cv::setIdentity(kf->measurementNoiseCov);
    kf->measurementNoiseCov *= 2.0;
    // kf->measurementNoiseCov.at<float>(3, 3) = 0.1;

    cv::setIdentity(kf->errorCovPost, cv::Scalar::all(100));

    // Define initial state
    kf->statePost = cv::Mat::zeros(10, 1, CV_32F);
}

int MOT3D::find_index_by_id(int id) {
    // find the index of the tracker by id
    for(int i = 0; i < trackers.size(); i++) {
        if(trackers[i].tracker_id == id) {
            return i;
        }
    }
    return -1;
}


void Tracker::predict(Eigen::Vector3f &position, Eigen::Vector3f &size, double &theta) {
    // predict
    cv::Mat prediction = kf->predict();

    position(0) = prediction.at<float>(0);
    position(1) = prediction.at<float>(1);
    position(2) = prediction.at<float>(2);

    theta = prediction.at<float>(3);
    size(0) = prediction.at<float>(4);
    size(1) = prediction.at<float>(5);
    size(2) = prediction.at<float>(6);
}

double within_range(double theta) {
    // Make sure the orientation is within a proper range

    if (theta >= M_PI) {
        theta -= 2 * M_PI;  // Make the theta still in the range
    }
    if (theta < -M_PI) {
        theta += 2 * M_PI;
    }

    return theta;
}

void orientation_correction(double &theta_update, double theta) {
    // Update orientation in propagated tracks and detected boxes so that they are within 90 degrees

    // Make the theta still in the range
    theta_update = within_range(theta_update);
    theta = within_range(theta);

    // If the angle of two theta is not acute angle, then make it acute
    if (std::abs(theta - theta_update) > M_PI / 2.0 && std::abs(theta - theta_update) < M_PI * 3 / 2.0) {
        theta_update += M_PI;
        theta_update = within_range(theta_update);
    }

    // Now the angle is acute: < 90 or > 270, convert the case of > 270 to < 90
    if (std::abs(theta - theta_update) >= M_PI * 3 / 2.0) {
        if (theta > 0) {
            theta_update += M_PI * 2;
        } else {
            theta_update -= M_PI * 2;
        }
    }
}

void Tracker::update(Eigen::Vector3f position, Eigen::Vector3f size, double theta){
    double theta_measure = theta;
    cv::Mat prediction = kf->predict();

    double theta_pre = prediction.at<float>(3);

    cv::Mat measurement(7, 1, CV_32F);
    measurement.at<float>(0) = position(0);
    measurement.at<float>(1) = position(1);
    measurement.at<float>(2) = position(2);

    measurement.at<float>(3) = theta_measure;
    measurement.at<float>(4) = size(0);
    measurement.at<float>(5) = size(1);
    measurement.at<float>(6) = size(2);

    cv::Mat estimated = kf->correct(measurement);

    // update the state
    position_estimate(0) = estimated.at<float>(0);
    position_estimate(1) = estimated.at<float>(1);
    position_estimate(2) = estimated.at<float>(2);

    theta_estimate = estimated.at<float>(3);
    size_estimate(0) = estimated.at<float>(4);
    size_estimate(1) = estimated.at<float>(5);
    size_estimate(2) = estimated.at<float>(6);

    delta_pos_estimate(0) = estimated.at<float>(7);
    delta_pos_estimate(1) = estimated.at<float>(8);
    delta_pos_estimate(2) = estimated.at<float>(9);
}

void MOT3D::predict(std::vector<BoundingBox> &bbox_predicted) {
    if(trackers.size() == 0)
        return;
    for(int i = 0; i < trackers.size(); i++) {
        Eigen::Vector3f position, size;
        double theta;
        trackers[i].predict(position, size, theta);

        BoundingBox bbox_pre(position, size, theta);

        bbox_predicted.emplace_back(bbox_pre);
    }
}

void MOT3D::update(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_predict) {
    for(int i = 0; i < assignment_predict.size(); i++) {
        if(assignment_predict[i] == -1)
            continue;
        trackers[i].update(bbox_observed[assignment_predict[i]].position, bbox_observed[assignment_predict[i]].size, bbox_observed[assignment_predict[i]].theta);
        // trackers[i].point_cloud = class_point_clouds_vector[assignment_predict[i]];
    }
}

void MOT3D::birthAndDeath(std::vector<BoundingBox> &bbox_observed, std::vector<int> assignment_observe, std::vector<int> assignment_predict) {
    // 把所有没有匹配上的观测作为新的tracker
    for(int i = 0; i < assignment_observe.size(); i++) {
        if(assignment_observe[i] == -1) {
            Tracker tracker;

            tracker.init(id_count++);
            tracker.hit_count ++;
            tracker.time_since_update = 0;

            Eigen::Vector3f position, size;
            double theta;
            position = bbox_observed[i].position;
            size = bbox_observed[i].size;
            theta = bbox_observed[i].theta;

            // x, y, z, theta, l, w, h, dx, dy, dz
            cv::Mat initial_state(10, 1, CV_32F);
            initial_state.at<float>(0) = position(0);
            initial_state.at<float>(1) = position(1);
            initial_state.at<float>(2) = position(2);
            initial_state.at<float>(3) = theta;
            initial_state.at<float>(4) = size(0);
            initial_state.at<float>(5) = size(1);
            initial_state.at<float>(6) = size(2);
            initial_state.at<float>(7) = 0;
            initial_state.at<float>(8) = 0;
            initial_state.at<float>(9) = 0;

            tracker.kf->statePost = initial_state;

            trackers.push_back(tracker);
            trackers[trackers.size() - 1].update(position, size, theta);
            // std::cout << "\033[33m" << "birth a new tracker: id " << id_count << "\033[0m" << std::endl;
        }
    }
    for(int i = 0; i < assignment_predict.size(); i++) {
        trackers[i].life_span ++;
        if(assignment_predict[i] == -1) {
            trackers[i].time_since_update ++;
        } else {
            trackers[i].time_since_update = 0;
            trackers[i].hit_count ++;
        }
        if(trackers[i].time_since_update > max_age) {
            trackers.erase(trackers.begin() + i);
            assignment_predict.erase(assignment_predict.begin() + i);
        }

        double vel_x = trackers[i].delta_pos_estimate(0) / time_between_two_update;
        double vel_y = trackers[i].delta_pos_estimate(1) / time_between_two_update;
        double vel = sqrt(vel_x * vel_x + vel_y * vel_y);
        trackers[i].static_time = (vel < vel_threshold) ? trackers[i].static_time + 1  : 0;
        if(trackers[i].static_time > max_static)
        {
            trackers[i].is_static = true;
        }
        else{
            trackers[i].is_static = false;
        }
    }
}

float boxArea(const BoundingBox& box) {
    return box.size(0) * box.size(1);
}

void DataAssociation(std::vector<BoundingBox> bbox_observed, std::vector<BoundingBox> bbox_predicted, std::vector<int> &assignment_predict,std::vector<int> &assignment_observe, std::vector<double> &cost_vector) {
    // calculate the distance between observed and predicted bbox
    int n_observed = bbox_observed.size();
    int n_predicted = bbox_predicted.size();

    // calculate the distance between observed and predicted bbox
    std::vector<std::vector<double>> cost_matrix(n_predicted, std::vector<double>(n_observed, 1000.0));

    std::vector<std::vector<double>> dist1_matrix(n_predicted, std::vector<double>(n_observed, 1000.0));
    std::vector<std::vector<double>> dist2_matrix(n_predicted, std::vector<double>(n_observed, 1000.0));


    for (int i = 0; i < n_predicted; i++) {
        for (int j = 0; j < n_observed; j++) {
            Eigen::Vector3f diff = bbox_predicted[i].position - bbox_observed[j].position;

            // std::cout << "diff: " << diff << std::endl;

            Eigen::Vector3f diff_size = bbox_predicted[i].size - bbox_observed[j].size;

            float distance = diff.norm();

            cost_matrix[i][j] = sqrt(diff(0) * diff(0) + diff(1) * diff(1));

            dist1_matrix[i][j] = diff.norm();
            dist2_matrix[i][j] = diff_size.norm();
        }
    }


    if (n_predicted == 0 || n_observed == 0) {
        return;
    }

    // solve the assignment problem
    // HungarianAlgorithm hungarian;
    // vector<int> Assignment;
    // double cost = hungarian.Solve(cost_matrix, Assignment);

    GreedyAssignmentAlgorithm greedy;

    vector<int> Assignment(n_predicted, -1);
    double cost = Solve(cost_matrix, Assignment);
    

    double distance_threshold = 3.0;
    double shape_threshold = 1.0;

    cost_vector = std::vector<double>(n_predicted, 10000);


    for (int i = 0; i < n_predicted; i++) {
        if(Assignment[i] == -1)
            continue;
        if(dist1_matrix[i][Assignment[i]] < distance_threshold) {
            assignment_predict[i] = Assignment[i];

            cost_vector[i] = cost_matrix[i][Assignment[i]];

            // std::cout << "cost: " << dist1_matrix[i][Assignment[i]] << " " << dist2_matrix[i][Assignment[i]] << std::endl;

            if(Assignment[i] < n_observed)
                assignment_observe[Assignment[i]] = i;
        }
    }
}