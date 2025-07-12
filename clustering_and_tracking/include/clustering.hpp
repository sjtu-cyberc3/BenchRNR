#ifndef _CLUSTERING_HPP
#define _CLUSTERING_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>
#include "../include/utilis.h"



void EuclideanCluster(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices>& cluster_indices, bool is_special_case=false);
void DBSCANCluster(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices> &cluster_indices);

void EnsembleCluster(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices> &cluster_indices);

void ProcessClusters(
    pcl::PointCloud<pcl::PointXYZI>& dynamic_cloud_sum,
    visualization_msgs::MarkerArray& marker_array,
    std::vector<BoundingBox>& boundingBoxes,
    int& marker_id, pcl::PointCloud<pcl::PointXYZI>& dynamic_cloud_clusterColored, pcl::PointCloud<pcl::PointXYZI>& current_cloud);

#endif