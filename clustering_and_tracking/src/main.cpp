#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <deque>
#include <unordered_map>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Eigen>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>
#include <std_msgs/Int32.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/PointCloud2.h>

#include <fstream>
#include "../include/ceres_icp.h"

#include <deque>

#include "../include/utilis.h"
#include "../include/tracker.hpp"
#include <unordered_map>
#include "../include/FEC_clustering.h"
#include "../include/clustering.hpp"

#include "../include/convex_optimize_fitting.h"
#include <sensor_msgs/NavSatFix.h>
#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H

#include <proj.h>
#include <proj_api.h>
#include <iomanip>
#include <proj/coordinateoperation.hpp>
#include <proj/crs.hpp>
#include <proj/io.hpp>
#include <proj/util.hpp>

#include <sensor_msgs/Imu.h>
#include "../include/Heading.h"
#include <ros/package.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace NS_PROJ::crs;
using namespace NS_PROJ::io;
using namespace NS_PROJ::operation;
using namespace NS_PROJ::util;


using namespace std;

ros::Subscriber sub_PointCloud2;
ros::Subscriber sub_MarkerArray_DL;
ros::Subscriber sub_trigger_msg;
ros::Subscriber sub_change_id_msg;


ros::Publisher pub_PointCloud2;
ros::Publisher pub_PointCloud2_clusterColored;

ros::Publisher pub_PointCloud2_origin;
ros::Publisher pub_PointCloud2_car;
ros::Publisher pub_PointCloud2_model;
ros::Publisher pub_PointCloud2_center;
ros::Publisher pub_ID_marker;
ros::Publisher pub_delete_marker;
ros::Publisher pub_vehicles_info;
ros::Publisher pub_obstacles_info;
ros::Publisher pub_MarkerArray;
ros::Publisher pub_MarkerArray_DL;
ros::Publisher pub_MarkerArray_KF;
ros::Publisher pub_markerArray_GT;
ros::Publisher pub_MakerrArray_entry;


ros::Publisher pub_PointCloud2_msg;


bool localization_initialized = false;
Eigen::Matrix3d global_tf_matrix = Eigen::Matrix3d::Identity();
Eigen::Vector2d main_dir;

ros::Time stamp_time;

pcl::PointCloud<pcl::PointXYZI> dynamic_model;
pcl::PointCloud<pcl::PointXYZI> last_frame_cloud;
pcl::PointCloud<pcl::PointXYZI> last_frame_cloud_2;


pcl::PointCloud<pcl::PointXYZI> pc_online;

std::string traj_num_str = "1";
std::string traj_num_str_output = "1";

int id_to_track = -1;

bool start_find_tracking_car = false;
bool find_tracking_car = false;
pcl::PointXYZI entry_center;
double entry_length;
double entry_width;
double entry_rot_angle;

pcl::PointCloud<pcl::PointXYZI> model_original;
pcl::PointCloud<pcl::PointXYZI> model_symmetry;

ofstream ofs_localization_bbox;  // Seg+obb
ofstream ofs_localization_convex;// Seg+convex
ofstream ofs_localization_DL;    // Deep learning: PV-RCNN
ofstream ofs_localization_model; // Register_loc
ofstream ofs_gps;           // RTK: Ground Truth
visualization_msgs::MarkerArray markerArray_id_output;
visualization_msgs::MarkerArray marker_array_entry;

// #define MULTI

#ifdef MULTI
void callback(const sensor_msgs::PointCloud2ConstPtr &msg_in1);
#else
void callback(const sensor_msgs::PointCloud2ConstPtr &msg_in1, const sensor_msgs::PointCloud2ConstPtr &msg_in2);
#endif

void callback_origin(const sensor_msgs::PointCloud2ConstPtr &msg_in1, const sensor_msgs::PointCloud2ConstPtr &msg_in2);
void callback_marker(const visualization_msgs::MarkerArray::ConstPtr &marker);

void odomCallback(const sensor_msgs::NavSatFix::ConstPtr &msg, const sensor_msgs::Imu::ConstPtr& msg_imu);

void start_find_tracking_car_callback(const std_msgs::Empty::ConstPtr &msg); 
void change_id_callback(const std_msgs::Int32::ConstPtr &msg);
void PCTransfor2D(pcl::PointCloud<pcl::PointXYZI> source, pcl::PointCloud<pcl::PointXYZI> &target, pcl::PointXYZI center_point, Eigen::MatrixXd obj_pca);

int car_frame = 0;


// Global variable for vehicle center
double model_center_x = 0.0;
double model_center_y = 0.0;

// Global variable for rear axle center of the vehicle
double model_zhou_x = 0.0;
double model_zhou_y = 0.0;

// Global variable for vehicle GPS localization result
double gps_center_x = 0.0;
double gps_center_y = 0.0;


MOT3D MOT;
double bias = 0.0;

pcl::PointCloud<pcl::PointXYZI> vehicle_points;
pcl::PointCloud<pcl::PointXYZI> static_model;

int frame_num_start = 0;
int model_init = 1;

// Number of point cloud frames used for clustering
int win_size = 4;

double localization_start_time = 0.0;
double localization_end_time = 0.0;

double localization_start_x = 0.0;
double localization_start_y = 0.0;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "clustering_and_tracking");
    ros::NodeHandle nh;

    nh.getParam("model_init", model_init);
    nh.getParam("traj_num_str",traj_num_str);

    int id_to_track_local = 0;
    nh.getParam("id_to_track", id_to_track_local);
    id_to_track = id_to_track_local;
    nh.getParam("bias", bias);

    bool auto_run = false;
    nh.getParam("auto", auto_run);

    //* -----------------------------------------------------------------------------------
    // Load trajectory parameters given traj_num_str
    if(auto_run) {
        getTrajectoryParams(std::string(ROOT_DIR) + "/data/auto_run/traj_start_info.txt", std::stoi(traj_num_str), localization_start_time, localization_end_time, model_init, localization_start_x, localization_start_y);
        std::cout << "Trajectory parameters loaded: traj_num = " << traj_num_str << " start_time = " << localization_start_time << ", end_time = " << localization_end_time << ", model_init = " << model_init << std::endl;
    }

    #ifdef MULTI
    sub_PointCloud2 = nh.subscribe<sensor_msgs::PointCloud2>("/dynamic_pointcloud_filtered_1", 100, &callback);
    #else
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc1_sub(nh, "/dynamic_pointcloud_filtered_1", 200);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc2_sub(nh, "/dynamic_pointcloud_filtered_2", 200);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(200), pc1_sub, pc2_sub); // queue size=10
    sync.registerCallback(boost::bind(&callback, _1, _2));
    #endif


    sub_MarkerArray_DL = nh.subscribe<visualization_msgs::MarkerArray>("/output_bounding_boxes", 100, &callback_marker);
    pub_PointCloud2 = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_pointcloud_sum", 100);
    pub_PointCloud2_origin = nh.advertise<sensor_msgs::PointCloud2>("/origin_pointcloud", 100);
    pub_PointCloud2_clusterColored = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_pointcloud_clusterColored", 100);
    pub_PointCloud2_car = nh.advertise<sensor_msgs::PointCloud2>("/vehicle_points", 100);


    pub_PointCloud2_model = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_pointcloud_model", 100);
    pub_PointCloud2_center = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_pointcloud_center", 100);
    pub_ID_marker = nh.advertise<visualization_msgs::MarkerArray>("object_markers", 100);
    pub_delete_marker = nh.advertise<visualization_msgs::Marker>("delete_markers",100);


    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_origin1_sub(nh, "/original_pointcloud1", 200);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_origin2_sub(nh, "/original_pointcloud2", 200);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync1(MySyncPolicy(200), pc_origin1_sub, pc_origin2_sub); // queue size=10
    sync1.registerCallback(boost::bind(&callback_origin, _1, _2));


    message_filters::Subscriber<sensor_msgs::NavSatFix> localization_sub(nh, "/Inertial/gps/fix", 200);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/Inertial/imu/data", 200);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy2;
    message_filters::Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(200), localization_sub, imu_sub); // queue size=10
    sync2.registerCallback(boost::bind(&odomCallback, _1, _2));



    sub_trigger_msg = nh.subscribe("trigger_topic", 10, &start_find_tracking_car_callback);
    sub_change_id_msg = nh.subscribe("change_id_topic",10,&change_id_callback);

    pub_MarkerArray = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_sum", 100);
    pub_MarkerArray_DL = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_DL", 100);
    pub_MarkerArray_KF = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_KF", 100);
    pub_markerArray_GT = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_STrack", 100);
    pub_MakerrArray_entry = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_entry", 100);

    // choose where the car is started from
    // 1： right entry
    // 2： left entry
    if(model_init == 1) {
        pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(ROOT_DIR) + "/data/car_model_changan_e4_1.pcd", dynamic_model);
    } else {
        pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(ROOT_DIR) + "/data/car_model_changan_e4_2.pcd", dynamic_model);
    }

    dynamic_model = VoxelDownsample(dynamic_model.makeShared(), 0.02);

    // Initialize entry-related information
    nh.getParam("entry_center_x",entry_center.x);
    nh.getParam("entry_center_y",entry_center.y);
    nh.getParam("entry_length", entry_length);
    nh.getParam("entry_width", entry_width);
    nh.getParam("entry_rot_angle", entry_rot_angle);

    #ifdef MULTI
    traj_num_str_output = traj_num_str + "m";  //n stands for multi
    
    int ring_arg = 0;
    nh.param<int>("rings", ring_arg, 128);
    traj_num_str_output = traj_num_str_output + "_" + std::to_string(ring_arg);  //n stands for multi
    
    #else 
    traj_num_str_output = traj_num_str + "n";  //n stands for non-repetitive
    #endif


    // Specify the file open path
    ofs_localization_model.open(std::string(ROOT_DIR) + "/data/regist/regist_"+traj_num_str_output+".txt", std::ios::out);
    ofs_gps.open(std::string(ROOT_DIR) + "/data/gt/gt_gps_"+traj_num_str_output+".txt");


    ofs_localization_convex.open(std::string(ROOT_DIR) + "/data/detect_convex/detect_"+traj_num_str_output+".txt");


    ofs_localization_DL.open(std::string(ROOT_DIR) + "/data/DL/DL_"+traj_num_str_output+".txt");
    ofs_localization_bbox.open(std::string(ROOT_DIR) + "/data/detect/detect_"+traj_num_str_output+".txt");

    ros::spin();

    return 0;
}


void start_find_tracking_car_callback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO(
        "---------------------------------------------------------------Initialization Trigger signal "
        "received!");
    start_find_tracking_car = true;
}

void change_id_callback(const std_msgs::Int32::ConstPtr &msg){
    ROS_INFO(
        "---------------------------------------------------------------Change ID signal "
        "received!");
    start_find_tracking_car = false;
    find_tracking_car = true;
    id_to_track = msg -> data;
}

void odomCallback(const sensor_msgs::NavSatFix::ConstPtr &msg, const sensor_msgs::Imu::ConstPtr& msg_imu)
{
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    double altitude = msg->altitude;

    double temp_z = msg_imu->orientation.z;
    double temp_w = msg_imu->orientation.w;

    double yaw = 2 * atan2(temp_z, temp_w);    

    double orientation_1 = cos(yaw);
    double orientation_2 = sin(yaw);

    int nUTMZone = (int)((longitude + 186.0) / 6.0);
    bool bNorth = latitude > 0 ? true : false;
    int m_nUTMZone = -1000;
    bool m_bNorth = false;
    projPJ m_pj_utm = nullptr;
    const char *wgs84 = "+proj=longlat +datum=WGS84 +no_defs "; // GPS所用坐标系,EPSG:4326
    projPJ m_pj_wgs84 = pj_init_plus(wgs84);
    if (m_nUTMZone != nUTMZone || m_bNorth != bNorth)
    {
        std::string qstrUTM = "+proj=utm +zone=" + std::to_string(nUTMZone);
        if (!bNorth)
        {
            qstrUTM += " +south ";
        }
        qstrUTM += " +datum=WGS84 +units=m +no_defs";
        if (m_pj_utm != nullptr)
        {
            pj_free(m_pj_utm);
            m_pj_utm = NULL;
        }
        std::string strUTM = qstrUTM;
        const char *pUTM = strUTM.c_str();

        m_pj_utm = pj_init_plus(pUTM);

        m_nUTMZone = nUTMZone;
        m_bNorth = bNorth;
    }
    double easting = longitude * DEG_TO_RAD;
    double northing = latitude * DEG_TO_RAD;
    pj_transform(m_pj_wgs84, m_pj_utm, 1, 1, &easting, &northing, nullptr);
    
    double  cosx = -0.8234426141;
    double sinx = -0.5673995503;

    double easting_zero = 2237088.2443682682;
    double northing_zero = 2628275.4305492351; 

    double zhou_x = cosx * easting + sinx * northing + easting_zero; 
    double zhou_y = - sinx * easting + cosx * northing + northing_zero;

    double temp_ori_x = orientation_1;
    double temp_ori_y = orientation_2;

    orientation_1 = cosx * temp_ori_x + sinx * temp_ori_y;
    orientation_2 = - sinx * temp_ori_x + cosx * temp_ori_y;

    // Transform RTK from rear axle to vehicle center using bias and principal direction.
    gps_center_x = zhou_x - bias * orientation_2;
    gps_center_y = zhou_y + bias * orientation_1;

    if (ofs_gps.is_open())
    {
        ofs_gps << std::fixed << std::setprecision(6) << msg->header.stamp<< " " << gps_center_x << " " << gps_center_y << " " << 0.0 << " " << 0 << " " << 0 << " " << orientation_1 << " " << orientation_2 << "\n";
    }
}


int frame_num = 0;

void callback_origin(const sensor_msgs::PointCloud2ConstPtr &msg_in1, const sensor_msgs::PointCloud2ConstPtr &msg_in2)
{
    // 合并多个点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::fromROSMsg(*msg_in2, *cur_cloud_ptr);
    *dynamic_cloud_ptr += *cur_cloud_ptr;

    sensor_msgs::PointCloud2 dynamic_msg_filtered;
    pcl::toROSMsg(*dynamic_cloud_ptr, dynamic_msg_filtered);

    dynamic_msg_filtered.header.stamp = msg_in1->header.stamp;
    dynamic_msg_filtered.header.frame_id = "rslidar";
    dynamic_msg_filtered.header.seq = 0;
    dynamic_msg_filtered.height = 1;
    dynamic_msg_filtered.width = dynamic_cloud_ptr->points.size();

    pub_PointCloud2_origin.publish(dynamic_msg_filtered);
}

double last_x = 0.0;

void callback_marker(const visualization_msgs::MarkerArray::ConstPtr &marker)
{
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::MarkerArray markerArray_output;

    markerArray = *marker;

    if(markerArray.markers.size() == 0) {
        return;
    }

    double min_distance = 10000.0;

    double nearest_x = 0.0;
    double nearest_y = 0.0;

    int min_dist_index = 0;

    if(localization_initialized){
        for (int i = 0; i < markerArray.markers.size(); ++i)
        {
            markerArray.markers[i].header.frame_id = "rslidar";

            double distance = sqrt(pow(markerArray.markers[i].pose.position.x - model_center_x, 2) + pow(markerArray.markers[i].pose.position.y - model_center_y, 2));

            if (distance < min_distance)
            {
                min_distance = distance;
                min_dist_index = i;
                nearest_x = markerArray.markers[i].pose.position.x;
                nearest_y = markerArray.markers[i].pose.position.y;
            }
        }
    }

    if(min_distance == 10000) {
        return;
    } else {
        double w = markerArray.markers[min_dist_index].pose.orientation.w;
        double z = markerArray.markers[min_dist_index].pose.orientation.z;

        double cosx = 1 - 2 * z * z;
        double sinx = 2 * w * z;

    // Only consider those within 2 meters of the ground truth as valid detection results
    double distance = sqrt(pow(markerArray.markers[min_dist_index].pose.position.x - model_center_x, 2) + pow(markerArray.markers[min_dist_index].pose.position.y - model_center_y, 2));
    if (distance > 2.0)
    {
        return;
    }

    if (ofs_localization_DL.is_open()) {
        ofs_localization_DL << markerArray.markers[0].header.stamp << " " << nearest_x << " " << nearest_y << " " << 0.0 << " " << 0 << " " << 0 << " " << cosx << " " << sinx <<"\n";
    }


    }

    pub_MarkerArray_DL.publish(markerArray_output);
}

ros::Time start_time;

// 滑窗保存多帧点云
std::deque<pcl::PointCloud<pcl::PointXYZI>> pc_deque;

double average_dbscan_time = 0.0;
double average_all_time = 0.0;
double average_cicp_time = 0.0;
double average_pure_cicp_time = 0.0;

#ifdef MULTI
void callback(const sensor_msgs::PointCloud2ConstPtr &msg_in1)
#else
void callback(const sensor_msgs::PointCloud2ConstPtr &msg_in1, const sensor_msgs::PointCloud2ConstPtr &msg_in2)
#endif
{
    double all_start = clock();

    stamp_time = msg_in1->header.stamp;
    pcl::PointCloud<pcl::PointXYZI> model_center_axis;

    visualization_msgs::MarkerArray marker_array;

    pcl::PointXYZI car_center_gps;
    car_center_gps.x = gps_center_x;
    car_center_gps.y = gps_center_y;
    car_center_gps.z = 0;
    car_center_gps.intensity = 1000;
    model_center_axis.push_back(car_center_gps);

    frame_num++;

    // combine the point clouds from two lidars
    pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::fromROSMsg(*msg_in1, *cur_cloud_ptr);
    *dynamic_cloud_ptr += *cur_cloud_ptr;

    #ifndef MULTI
    pcl::fromROSMsg(*msg_in2, *cur_cloud_ptr);
    *dynamic_cloud_ptr += *cur_cloud_ptr;
    #endif

    double time_before_dbscan = clock();

    pcl::PointCloud<pcl::PointXYZI> dynamic_cloud_filtered;
    pcl::PointCloud<pcl::PointXYZI> dynamic_cloud_clusterColored;

    double before_filter = clock();
    dynamic_cloud_filtered = *dynamic_cloud_ptr;

    vector<vector<int>> cluster_index;
    pcl::PointCloud<pcl::PointXYZI> dynamic_cloud_sum;

    if(pc_deque.size() < win_size) {
        pc_deque.push_back(dynamic_cloud_filtered);
    } else {
        pc_deque.pop_front();
        pc_deque.push_back(dynamic_cloud_filtered);
    }

    for(int i = pc_deque.size()-1; i > -1; i--) {
        dynamic_cloud_sum += pc_deque[i];
    }


    dynamic_cloud_sum = VoxelDownsample(dynamic_cloud_sum.makeShared(), 0.15);

    int marker_id = 0;


    // 保存聚类出的包围盒
    std::vector<BoundingBox> boundingBoxes;
    // 输出id结果
    visualization_msgs::MarkerArray markerArray_GT;



    // clear marker_array
    visualization_msgs::Marker delete_bbox_marker;
    delete_bbox_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_bbox_marker.ns = "boundingbox";
    pub_delete_marker.publish(delete_bbox_marker);


    marker_array.markers.clear();


    ProcessClusters(
        dynamic_cloud_sum,
        marker_array,
        boundingBoxes,
        marker_id,dynamic_cloud_clusterColored, dynamic_cloud_filtered);

    std::vector<BoundingBox> bbox_predicted;
    MOT.predict(bbox_predicted);

    // tracking module: associate predicted bounding boxes with observed bounding boxes, thus tracking the targets accross frames
    std::vector<int> assignment_predict(bbox_predicted.size(), -1);
    std::vector<int> assignment_observe(boundingBoxes.size(), -1);

    std::vector<double> cost_vector;

    DataAssociation(boundingBoxes, bbox_predicted, assignment_predict, assignment_observe, cost_vector);

    //* -----------------------------------------------------------------------------
    // Determine the vehicle ID be tracked.

    // if is in automatic mode, and the time of start is reached
    // then find the nearest target of GT. This only runs once.
    if(localization_start_time != 0 && abs(stamp_time.toSec() - localization_start_time) < 0.1) {
        // find the nearset target of GT
        double min_distance = 10000.0;
        int min_index = -1;
        for(int i = 0; i < bbox_predicted.size(); i++) {
            if(assignment_predict[i] == -1) continue;
            double distance = sqrt(pow(bbox_predicted[i].position(0) - gps_center_x, 2) + pow(bbox_predicted[i].position(1) - gps_center_y, 2));
            if(distance < min_distance) {
                min_distance = distance;
                min_index = i;
            }
        }

        // 根据最小距离的目标ID来设置id_to_track
        if(min_index != -1) {
            id_to_track = MOT.trackers[min_index].tracker_id;
            find_tracking_car = true;
            start_find_tracking_car = false;
            std::cout << "ID to track: " << id_to_track << std::endl;
        }
    }

    // Process the tracking target vehicle ID entered via the terminal
    if (start_find_tracking_car == true && find_tracking_car == false) {
        ROS_INFO("----------start initialization");
        std::vector<pcl::PointXYZI> vertices(4);
        getEntryVertices(entry_center, entry_length, entry_width, entry_rot_angle, vertices);
        for (int i = 0; i < MOT.trackers.size(); i++) {
            std::cout << "-----width:"
                      << MOT.trackers[i].size_estimate(0) << std::endl;
            std::cout << "-----length:"
                      << MOT.trackers[i].size_estimate(1) << std::endl;
            if (MOT.trackers[i].size_estimate(0) > 8 || MOT.trackers[i].size_estimate(1) > 8) continue;
            if (MOT.trackers[i].size_estimate(0) < 0.8 || MOT.trackers[i].size_estimate(1) < 0.8) continue;

            id_to_track = MOT.trackers[i].tracker_id;
            std::cout << "-----find car to track: "
                        << id_to_track << std::endl;
            break;
        }
    }

    visualization_msgs::Marker delete_id_marker;
    delete_id_marker.action = visualization_msgs::Marker::DELETEALL;
    pub_delete_marker.publish(delete_id_marker);

    int marker_count = 0;

    for(int i = 0; i < assignment_predict.size(); i++) {
        if(assignment_predict[i] != -1) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "rslidar";
            marker.header.stamp = ros::Time::now();
            marker.ns = "id";
            marker.id = marker_count++;
            marker.lifetime = ros::Duration(0.1);
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = bbox_predicted[i].position(0);
            marker.pose.position.y = bbox_predicted[i].position(1);
            marker.pose.position.z = 3;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 1.0;
            marker.color.a = 1.0;
            if(MOT.trackers[i].tracker_id == id_to_track){
                // =target id is visualized in blue
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }else{
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }

            marker.text = "id:" + std::to_string(MOT.trackers[i].tracker_id);
            markerArray_id_output.markers.push_back(marker);
        }
    }
    pub_ID_marker.publish(markerArray_id_output);


    double time_before_cicp = clock();


    for(int i = 0; i < assignment_predict.size(); i++) {
            if(MOT.trackers[i].tracker_id == id_to_track) {
                int my_index = MOT.find_index_by_id(id_to_track);
                BoundingBox my_bbox = boundingBoxes[assignment_predict[my_index]];

                // Perform ICP using only the point cloud from the current frame
                pcl::PointCloud<pcl::PointXYZI> cur_vehicle_cloud;

                // Extract the portion of the current frame's point cloud that lies within the bounding box of the current target
                cur_vehicle_cloud = extractPointsInsideBoundingBox(dynamic_cloud_filtered.makeShared(), my_bbox);
               

                if(cur_vehicle_cloud.size() == 0) {
                    break;
                }

                if(!localization_initialized && MOT.trackers[i].hit_count > 5) {
                    localization_initialized = true;
                    frame_num_start = frame_num;
                    break;
                }
                //* -----------------------------------------------------------------------------
                // draw ground truth bounding box
                double timestamp = stamp_time.toSec();
                BoundingBox gtbbox;
                drawGTBBox(markerArray_GT, timestamp, gtbbox, traj_num_str);   

                //* -----------------------------------------------------------------------------
                // Seg+obb
                Eigen::MatrixXd obj_pca_obb;
                obj_pca_obb = pca(cur_vehicle_cloud.makeShared());
                
                double x_obb = obj_pca_obb(0, 0);
                double y_obb = obj_pca_obb(0, 1);
                double heading_obb = atan2(y_obb, x_obb);
                
                double length_obb;
                double width_obb;
                double height_obb;
                
                pcl::PointXYZI point_zhou_obb;
                pcl::PointXYZI center_point_obb;

                marker_id++;
                MyBoundingBox(cur_vehicle_cloud, center_point_obb, obj_pca_obb, marker_array, marker_id,
                              length_obb, width_obb, height_obb, ros::Time::now(),
                              point_zhou_obb, 3, 0, 0);   
                // Seg_obb results
                if (ofs_localization_bbox.is_open())  {
                    ofs_localization_bbox << stamp_time << " " << center_point_obb.x << " " << center_point_obb.y << " " << 0 << " " << 0 << " " << 0 << " " << cos(heading_obb) << " " << sin(heading_obb) << "\n";
                }                   
                
                //* -----------------------------------------------------------------------------  
                // Seg+convex: Use a convex-hull-based method to generate bounding boxes for point clouds.
                // cite: https://github.com/HMX2013/CH-MOA-ROS.git
                double convex_yaw;
                Eigen::Vector3d convex_position;
                Eigen::Vector3d convex_size;
     
                ConvexOptimizeFitting app;
                app.bbox_fitting(cur_vehicle_cloud, convex_position, convex_size, convex_yaw);

                // Seg_convex results
                if (ofs_localization_convex.is_open())  {
                    ofs_localization_convex << stamp_time << " " << convex_position(0) << " " << convex_position(1) << " " << 0 << " " << 0 << " " << 0 << " " << cos(convex_yaw) << " " << sin(convex_yaw) << "\n";
                }
                //* -----------------------------------------------------------------------------
               // Apply hierarchical downsampling to the vehicle point cloud.
               double voxel_size = 0.1;
               if(cur_vehicle_cloud.size() > 300) {
                    voxel_size = 0.4;
                } else if(cur_vehicle_cloud.size() > 200) {
                    voxel_size = 0.2;
                } else if(cur_vehicle_cloud.size() < 100) {
                    voxel_size = 0.0;
                }
                if(voxel_size > 0.005) {
                    cur_vehicle_cloud = VoxelDownsample(cur_vehicle_cloud.makeShared(), voxel_size);
                }

                //* -----------------------------------------------------------------------------
                // Register_loc
                vehicle_points = cur_vehicle_cloud;
                double time_before_pure_icp = clock();
                my_icp(cur_vehicle_cloud, dynamic_model, global_tf_matrix);
                double time_after_pure_icp = clock();
                average_pure_cicp_time = (average_pure_cicp_time * (frame_num - 1) + (time_after_pure_icp - time_before_pure_icp) / CLOCKS_PER_SEC) / frame_num;
                
                std::cout << "\033[33mcurrent pure cicp time: " << (time_after_pure_icp - time_before_pure_icp) / CLOCKS_PER_SEC << "\033[0m" << std::endl;
                std::cout << "\033[33maverage pure cicp time: " << average_pure_cicp_time << "\033[0m" << std::endl;

                pcl::PointXYZI model_center;
                // Compute the principal orientation of the matched template using PCA
                Eigen::MatrixXd model_pca = pca(dynamic_model.makeShared());
                double len, wid, height, heading;
                height = 0;
                pcl::PointXYZI point_zhou;
                MyBoundingBox(dynamic_model, model_center, model_pca, marker_array, marker_id, len, wid, height, ros::Time::now(), point_zhou, 1, bias, 1999);
                main_dir(0) = model_pca(0,0);
                main_dir(1) = model_pca(0,1);
                heading = atan2(model_pca(0,1),model_pca(0,0));

                // register_loc results
                if (ofs_localization_model.is_open()) {
                    ofs_localization_model << stamp_time  << " " <<  model_center.x << " " <<   model_center.y << " " << 0 << " " << 0 << " " << cur_vehicle_cloud.size() << " " << main_dir(0)<< " " <<main_dir(1)<< "\n";             
                } 

                pcl::PointXYZI car_center_model;
                car_center_model.x = model_center.x;
                car_center_model.y = model_center.y;
                car_center_model.z = 0;
                car_center_model.intensity = 100;
                point_zhou.intensity = 100;
                model_center_axis.push_back(car_center_model);
                model_center_x = model_center.x;
                model_center_y = model_center.y;

                model_zhou_x = point_zhou.x;
                model_zhou_y = point_zhou.y;
            }
    }


    // update kalman filter with the observed bounding boxes
    MOT.update(boundingBoxes, assignment_predict);

    // Use the Birth & Death Manager to handle newly appeared and disappeared targets
    MOT.birthAndDeath(boundingBoxes, assignment_observe, assignment_predict);

    double time_after_cicp = clock();

    average_cicp_time = (average_cicp_time * (frame_num - 1) + (time_after_cicp - time_before_cicp) / CLOCKS_PER_SEC) / frame_num;

    // std::cout << "\033[33mcurrent cicp time: " << (time_after_cicp - time_before_cicp) / CLOCKS_PER_SEC << "\033[0m" << std::endl;
    // std::cout << "\033[33maverage cicp time: " << average_cicp_time << "\033[0m" << std::endl;

    pub_markerArray_GT.publish(markerArray_GT);
    pub_MarkerArray.publish(marker_array);

    sensor_msgs::PointCloud2 model_message;
    pcl::toROSMsg(dynamic_model, model_message);

    model_message.header.stamp = msg_in1->header.stamp;
    model_message.header.frame_id = "rslidar";
    model_message.header.seq = 0;
    model_message.height = 1;
    model_message.width = dynamic_model.size();

    pub_PointCloud2_model.publish(model_message);

    sensor_msgs::PointCloud2 car_message;
    pcl::toROSMsg(vehicle_points, car_message);

    car_message.header.stamp = msg_in1->header.stamp;
    car_message.header.frame_id = "rslidar";
    car_message.header.seq = 0;
    car_message.height = 1;
    car_message.width = vehicle_points.size();

    pub_PointCloud2_car.publish(car_message);

    sensor_msgs::PointCloud2 center_message;
    pcl::toROSMsg(model_center_axis, center_message);

    center_message.header.stamp = msg_in1->header.stamp;
    center_message.header.frame_id = "rslidar";
    center_message.header.seq = 0;
    center_message.height = 1;
    center_message.width = model_center_axis.points.size();

    pub_PointCloud2_center.publish(center_message);

    sensor_msgs::PointCloud2 dynamic_msg_filtered;
    pcl::toROSMsg(dynamic_cloud_sum, dynamic_msg_filtered);

    dynamic_msg_filtered.header.stamp = msg_in1->header.stamp;
    dynamic_msg_filtered.header.frame_id = "rslidar";
    dynamic_msg_filtered.header.seq = 0;
    dynamic_msg_filtered.height = 1;
    dynamic_msg_filtered.width = dynamic_cloud_sum.points.size();

    pub_PointCloud2.publish(dynamic_msg_filtered);


    pcl::toROSMsg(dynamic_cloud_clusterColored, dynamic_msg_filtered);

    dynamic_msg_filtered.header.stamp = msg_in1->header.stamp;
    dynamic_msg_filtered.header.frame_id = "rslidar";
    dynamic_msg_filtered.header.seq = 0;
    dynamic_msg_filtered.height = 1;
    dynamic_msg_filtered.width = dynamic_cloud_clusterColored.points.size();

    pub_PointCloud2_clusterColored.publish(dynamic_msg_filtered);

    // 可视化entry形状
    visualization_msgs::Marker delete_entry_marker;
    delete_entry_marker.action = visualization_msgs::Marker::DELETEALL;
    delete_entry_marker.ns = "entry";
    pub_delete_marker.publish(delete_entry_marker);
    marker_array_entry.markers.clear();

    // 可视化entry形状
    create_entry_box(marker_array_entry, entry_center, entry_length, entry_width, entry_rot_angle, stamp_time);
    pub_MakerrArray_entry.publish(marker_array_entry);


    double after_bbx = clock();

    double all_end = clock();

    std::cout << "tracking all time: " << (all_end - all_start) / CLOCKS_PER_SEC << std::endl;
    average_all_time = (average_all_time * (frame_num - 1) + (all_end - all_start) / CLOCKS_PER_SEC) / frame_num;

    std::cout << "\033[32maverage tracking time: " << average_all_time << "\033[0m" << std::endl;

    //* -----------------------------------------------------------------------------------
    // automatic shut down
    if(localization_end_time != 0 && abs(stamp_time.toSec() - localization_end_time) < 0.1) {
        std::cout << "End time reached, shutting down the clustering&tracking node." << std::endl;
        ofs_localization_model.close();
        ofs_localization_convex.close();
        ofs_localization_bbox.close();
        ofs_localization_DL.close();
        ofs_gps.close();
        ros::shutdown();
    }
}


void PCTransfor2D(pcl::PointCloud<pcl::PointXYZI> source, pcl::PointCloud<pcl::PointXYZI> &target, pcl::PointXYZI center_point, Eigen::MatrixXd obj_pca)
{
    for (int i = 0; i < source.size(); i++)
    {
        pcl::PointXYZI temp_point = source[i];

        double x1 = source[i].x - center_point.x;
        double y1 = source[i].y - center_point.y;

        temp_point.x = obj_pca(0, 0) * x1 + obj_pca(0, 1) * y1;
        temp_point.y = obj_pca(1, 0) * x1 + obj_pca(1, 1) * y1;

        target.push_back(temp_point);
    }
}