#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <deque>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>

// #define MULTI

using namespace std;

#ifdef MULTI
    #define U_PIXEL 1800
    #define V_PIXEL 1800
    #define RES 0.2
#else             // Avia  
    #define U_PIXEL 704
    #define V_PIXEL 772
    #define RES 0.1   
#endif



pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;


ros::Subscriber sub_PointCloud2;
ros::Publisher pub_PointCloud2_filtered;

ros::Publisher pub_PointCloud2_static;
ros::Publisher pub_PointCloud2_origin;

#ifdef MULTI
void PL2Callback(const sensor_msgs::PointCloud2ConstPtr &lidarMsg);
#else
void PL2Callback(const livox_ros_driver::CustomMsgConstPtr &msg_in);
#endif

// to save the background image
vector<vector<double>> background_map(U_PIXEL, vector<double>(V_PIXEL, 0));
// to save the background point cloud
vector<vector<pcl::PointXYZI>> background_cloud(U_PIXEL, vector<pcl::PointXYZI>(V_PIXEL));

pcl::PointCloud<pcl::PointXYZI> static_cloud;


int frame_count = 0;

Eigen::Matrix4d R_trans;
Eigen::Matrix3d R_Mat;
Eigen::Vector4d trans_vector;

std::string background_file_name = "";
std::string livox_input_topic = "";

std::string foreground_pc_topic = "";
std::string depth_image_name = "";
std::string num_str = "";

#ifdef MULTI
std::vector<int> ring_vec_global;
#endif


int main(int argc, char **argv)
{
    ros::init(argc, argv, "foreground_segmentation");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh.param<std::string>("background/background_file_name", background_file_name, "background_livox_1.pcd");
    nh.param<std::string>("background/depth_image_name", depth_image_name, "/background_depth_1.txt");
    nh.param<std::string>("subandpub/sub_custom_topic", livox_input_topic, "/livox2");

    nh.param<std::string>("subandpub/foreground_pc_topic", foreground_pc_topic, "/dynamic_pointcloud_filtered_2");
    nh.param<std::string>("subandpub/num", num_str, "2");

    #ifdef MULTI
    sub_PointCloud2 = nh.subscribe<sensor_msgs::PointCloud2>(livox_input_topic, 100, &PL2Callback);
    #else
    sub_PointCloud2 = nh.subscribe<livox_ros_driver::CustomMsg>(livox_input_topic, 100, &PL2Callback);
    #endif

    pub_PointCloud2_filtered = nh.advertise<sensor_msgs::PointCloud2>(foreground_pc_topic, 100);
    pub_PointCloud2_static = nh.advertise<sensor_msgs::PointCloud2>("/static_pointcloud" + num_str, 100);
    pub_PointCloud2_origin = nh.advertise<sensor_msgs::PointCloud2>("/origin_pointcloud" + num_str, 100);

    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    double d_x = 0;
    double d_y = 0;
    double d_z = 0;

    nh.param<double>("matrix/delta_x", d_x, 0);
    nh.param<double>("matrix/delta_y", d_y, 0);
    nh.param<double>("matrix/delta_z", d_z, 0);

    nh.param<double>("matrix/yaw", yaw, 0);
    nh.param<double>("matrix/pitch", pitch, 0);
    nh.param<double>("matrix/roll", roll, 0);

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    R_Mat = yawAngle * pitchAngle * rollAngle;
    trans_vector << d_x, d_y, d_z, 1;
    R_trans.block(0, 0, 3, 3) = R_Mat;
    R_trans.col(3) = trans_vector;

    // load background point cloud
    pcl::io::loadPCDFile<pcl::PointXYZI>(std::string(ROOT_DIR)+"/data/" + background_file_name, static_cloud);
    std::cout << "Loaded " << static_cloud.size() << " data points from " << background_file_name << std::endl;

    // load background depth image
    ifstream ifs(std::string(ROOT_DIR)+"/data/" + depth_image_name, ios::in);
    if (!ifs.is_open())
    {
        std::cerr << "Error: Unable to open file." << std::endl;
        return -1;
    }

    for (int u = 0; u < U_PIXEL; u++)
    {
        for (int v = 0; v < V_PIXEL; v++)
        {
            ifs >> background_map[u][v];
        }
    }

    // KdTree init
    kdtree.setInputCloud(static_cloud.makeShared());

    #ifdef MULTI
    int ring_arg = 0;

    nh.param<int>("rings", ring_arg, 128);

    std::cout << "++++++++++++++++++++++++++++++++++++++++ Rings: " << ring_arg << std::endl;

    // 读入ring_txt
    std::ifstream ring_file(std::string(ROOT_DIR) + "/data/ring_" + std::to_string(ring_arg) + ".txt");
    if (ring_file.is_open()) {
        std::string line;
        while (std::getline(ring_file, line)) {
            int ring_num = std::stoi(line);
            ring_vec_global.push_back(ring_num);
        }
        ring_file.close();
    } else {
        std::cerr << "Unable to open file" << std::endl;
        std::cout << std::string(ROOT_DIR) + "/data/ring_" + std::to_string(ring_arg) + ".txt" << std::endl;
    }
    #endif

    ros::spin();

    return 0;
}


double average_all_time = 0.0;
double maximum_all_time = 0.0;
#ifdef MULTI
void PL2Callback(const sensor_msgs::PointCloud2ConstPtr &msg_in)
#else
void PL2Callback(const livox_ros_driver::CustomMsgConstPtr &msg_in)
#endif
{
    cv::Mat image_foreground(U_PIXEL, V_PIXEL, CV_64FC1);
    double all_start = clock();

    // Store the foreground point cloud obtained through background filtering.
    pcl::PointCloud<pcl::PointXYZI> dynamic_cloud_filtered; 

    //Store the raw point cloud transformed into the world coordinate system.
    pcl::PointCloud<pcl::PointXYZI> origin_cloud;

    #ifdef MULTI
    pcl::PointCloud<pcl::PointXYZI> raw_pointcloud;
    pcl::fromROSMsg(*msg_in, raw_pointcloud);

    for (int i = 0; i < raw_pointcloud.size(); i++) {
        pcl::PointXYZI cur_point;
        cur_point.x = raw_pointcloud[i].x;
        cur_point.y = raw_pointcloud[i].y;
        cur_point.z = raw_pointcloud[i].z;
        cur_point.intensity = raw_pointcloud[i].intensity;

    #else  
    for (int i = 0; i < msg_in->point_num; i++) {
        pcl::PointXYZI cur_point;
        cur_point.x = msg_in->points[i].x;
        cur_point.y = msg_in->points[i].y;
        cur_point.z = msg_in->points[i].z;
        cur_point.intensity = msg_in->points[i].reflectivity;
    #endif

        double depth = sqrt(pow(cur_point.x, 2) + pow(cur_point.y, 2) + pow(cur_point.z, 2));
        double theta_u = atan2(cur_point.z, sqrt(pow(cur_point.x, 2) + pow(cur_point.y, 2)));
        double theta_v = atan2(cur_point.y, cur_point.x);

        theta_u = theta_u / M_PI * 180.0;
        theta_v = theta_v / M_PI * 180.0;


        #ifdef MULTI
        int ring_u = int(theta_u*10);
        // Keep only the points within the current line number
        if(std::find(ring_vec_global.begin(), ring_vec_global.end(), ring_u) == ring_vec_global.end()) {
            continue;
        }
        #endif

#ifdef MULTI
        theta_u = 180 - theta_u;
        theta_v = 60 - theta_v;
#else 
        // AVIA
        theta_u = 35.2 - theta_u;
        theta_v = 38.6 - theta_v;
#endif
        int u = floor(theta_u / RES);
        int v = floor(theta_v / RES);

        if (u < 0 || u > U_PIXEL - 1 || v < 0 || v > V_PIXEL - 1)
            continue;

        // Transform the point cloud from the LiDAR coordinate system to the world coordinate system
        Eigen::Vector4d cur_point_vec;
        cur_point_vec << cur_point.x, cur_point.y, cur_point.z, 1;
        cur_point_vec = R_trans * cur_point_vec;

        pcl::PointXYZI origin_point;
        origin_point.x = cur_point_vec(0);
        origin_point.y = cur_point_vec(1);
        origin_point.z = cur_point_vec(2);
        origin_point.intensity = cur_point.intensity;

        // filter out unnecessary points
        if (origin_point.z < 0.03 || depth <= 2.5 || depth > 100 || origin_point.z > 2) {
            continue;
        }
        
        origin_cloud.push_back(origin_point);



        double depth_threshold = 0.3;

        if (background_map[u][v] - depth < depth_threshold || background_map[u][v] == 0) {                                            
            continue;
        }

        // Nearest neighbor search to filter points based on the background point cloud
        int K = 1;
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        if (kdtree.nearestKSearch(origin_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) < 0)
        {
            continue;
        }

        // filter points based on the squared distance
        if(pointNKNSquaredDistance[0] > 0.09)
        {
            dynamic_cloud_filtered.push_back(origin_point);
        }
    }

   
    // -----------------------------------------------------------------------------------

    double before_filter = clock();

    // publish the foregorund point cloud
    sensor_msgs::PointCloud2 dynamic_msg_filtered;
    pcl::toROSMsg(dynamic_cloud_filtered, dynamic_msg_filtered);

    dynamic_msg_filtered.header.stamp = msg_in->header.stamp;
    dynamic_msg_filtered.header.frame_id = "rslidar";
    dynamic_msg_filtered.header.seq = 0;
    dynamic_msg_filtered.height = 1;
    dynamic_msg_filtered.width = dynamic_cloud_filtered.points.size();

    pub_PointCloud2_filtered.publish(dynamic_msg_filtered);

    // visualize the background point cloud  
    sensor_msgs::PointCloud2 background_msg;
    pcl::toROSMsg(static_cloud, background_msg);

    background_msg.header.stamp = msg_in->header.stamp;
    background_msg.header.frame_id = "rslidar";
    background_msg.header.seq = 0;
    background_msg.height = 1;
    background_msg.width = static_cloud.points.size();

    pub_PointCloud2_static.publish(background_msg);

    frame_count++;
    double all_end = clock();
    std::cout << "fore all time: " << (all_end - all_start) / CLOCKS_PER_SEC << std::endl;

    average_all_time = (average_all_time * (frame_count - 1) + (all_end - all_start) / CLOCKS_PER_SEC) / frame_count;

    maximum_all_time = std::max(maximum_all_time, (all_end - all_start) / CLOCKS_PER_SEC);

    std::cout << "\033[32maverage foreground time: " << average_all_time << "\033[0m" << std::endl;
    // std::cout << "\033[32mmaximum foreground time: " << maximum_all_time << "\033[0m" << std::endl;
}