#include "../include/clustering.hpp"


void EuclideanCluster(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices>& cluster_indices, bool is_special_case) {
    
    pcl::PointCloud<pcl::PointXYZI> cloud_2d;

    for (int i = 0; i < input.size(); i++) {
        pcl::PointXYZI point;
        #ifdef MULTI
        point.x = input[i].x * 0.3; //0.4
        #else
        point.x = input[i].x;
        #endif
        point.y = input[i].y;
        point.z = 0;
        point.intensity = input[i].intensity;
        cloud_2d.push_back(point);
    }
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr dytree(new pcl::search::KdTree<pcl::PointXYZI>());
    dytree->setInputCloud(cloud_2d.makeShared());

    double distance_threshold = 0.5;

    #ifdef MULTI
    distance_threshold = 0.6; //0.4
    #endif

    if(is_special_case) {
        distance_threshold = 0.22; // 特殊情况，聚类容忍度设置为0.1
    }

    // 欧式聚类
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(distance_threshold); // 设置聚类容忍度，即点与点之间的最大距离
    ec.setMinClusterSize(20);    // 设置最小聚类大小
    ec.setMaxClusterSize(25000); // 设置最大聚类大小
    ec.setSearchMethod(dytree);  // 设置搜索方法
    ec.setInputCloud(cloud_2d.makeShared());
    ec.extract(cluster_indices);
}

void DBSCANCluster(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices> &cluster_indices)
{
    double r = 0.30;
    int size = 30;

    pcl::PointCloud<pcl::PointXYZI> cloud_2d;

    for (int i = 0; i < input.size(); i++) {
        pcl::PointXYZI point;
        point.x = input[i].x;
        point.y = input[i].y;
        point.z = 0;
        point.intensity = input[i].intensity;
        cloud_2d.push_back(point);
    }
    

    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(cloud_2d.makeShared());
    std::vector<bool> cloud_processed(cloud_2d.size(), false);

    for (size_t i = 0; i < cloud_2d.points.size(); ++i)
    {
        if (cloud_processed[i])
        {
            continue;
        }

        std::vector<int> seed_queue;
        // 检查近邻数是否大于给定的size（判断是否是核心对象）
        std::vector<int> indices_cloud;
        std::vector<float> dists_cloud;
        if (tree.radiusSearch(cloud_2d.points[i], r, indices_cloud, dists_cloud) >= size)
        {
            seed_queue.push_back(i);
            cloud_processed[i] = true;
        }
        else
        {
            continue;
        }

        int seed_index = 0;
        while (seed_index < seed_queue.size())
        {
            std::vector<int> indices;
            std::vector<float> dists;
            if (tree.radiusSearch(cloud_2d.points[seed_queue[seed_index]], r, indices, dists) < size) // 函数返回值为近邻数量
            {
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (cloud_processed[indices[j]])
                {
                    continue;
                }
                seed_queue.push_back(indices[j]);
                cloud_processed[indices[j]] = true;
            }
            ++seed_index;
        }

        pcl::PointIndices cluster;
        cluster.indices = seed_queue;
        cluster_indices.push_back(cluster);
    }
}


void EnsembleCluster(pcl::PointCloud<pcl::PointXYZI>& input, std::vector<pcl::PointIndices> &cluster_indices) {
    std::vector<pcl::PointIndices> cluster_indices_dbscan;

    DBSCANCluster(input, cluster_indices_dbscan);

    pcl::PointCloud<pcl::PointXYZI> cloud_good;
    for (int i = 0; i < cluster_indices_dbscan.size(); i++) {
        for (int j = 0; j < cluster_indices_dbscan[i].indices.size(); j++) {
            cloud_good.push_back(input[cluster_indices_dbscan[i].indices[j]]);
        }
    }

    input = cloud_good;

    EuclideanCluster(cloud_good, cluster_indices);
}


void ProcessClusters(
    pcl::PointCloud<pcl::PointXYZI>& dynamic_cloud_sum,
    visualization_msgs::MarkerArray& marker_array,
    std::vector<BoundingBox>& boundingBoxes,
    int& marker_id, pcl::PointCloud<pcl::PointXYZI>& dynamic_cloud_clusterColored, pcl::PointCloud<pcl::PointXYZI>& current_cloud)
{
    std::vector<pcl::PointIndices> cluster_indices;
    EuclideanCluster(dynamic_cloud_sum, cluster_indices);

    for (const auto& indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZI> class_point_cloud;

        for (const auto& idx : indices.indices)
            class_point_cloud.push_back(dynamic_cloud_sum.points[idx]);

        if (class_point_cloud.size() <= 10)
            continue;


        Eigen::MatrixXd obj_pca = pca(class_point_cloud.makeShared());
        double heading = atan2(obj_pca(0, 1), obj_pca(0, 0));

        double length, width, height;
        pcl::PointXYZI center_point, point_zhou;
        MyBoundingBox(class_point_cloud, center_point, obj_pca, marker_array, marker_id, length, width, height, ros::Time::now(), point_zhou, 0);
        marker_id++;

        double r = (rand() % 100) / 100.0;

        if(length * width < 14.0) {
            for(int j = 0; j < class_point_cloud.size(); j++) {
                pcl::PointXYZI myPoint = class_point_cloud[j];
    
                // 是否用强度来表示类别
                myPoint.intensity = r;
    
                dynamic_cloud_clusterColored.push_back(myPoint);
            }
        }


        // 再次细分过大的聚类
        if (length * width > 14.0)
        {
            std::vector<pcl::PointIndices> cluster_indices1;
            EuclideanCluster(class_point_cloud, cluster_indices1, 0.5);  // 更小容差

            for (const auto& sub_indices : cluster_indices1)
            {
                pcl::PointCloud<pcl::PointXYZI> cur_point_cloud;
                for (const auto& idx1 : sub_indices.indices)
                    cur_point_cloud.push_back(class_point_cloud[idx1]);

                if (cur_point_cloud.size() <= 10)
                    continue;

                Eigen::MatrixXd obj_pca1 = pca(cur_point_cloud.makeShared());
                double heading1 = atan2(obj_pca1(0, 1), obj_pca1(0, 0));

                double length1, width1, height1;
                pcl::PointXYZI center_point1;
                MyBoundingBox(cur_point_cloud, center_point1, obj_pca1, marker_array, marker_id, length1, width1, height1, ros::Time::now(), point_zhou, 0);
                marker_id++;

                double r = (rand() % 100) / 100.0;


                for(int j = 0; j < class_point_cloud.size(); j++) {
                    pcl::PointXYZI myPoint = class_point_cloud[j];
                    // 是否用强度来表示类别

                    myPoint.intensity = r;
        
                    dynamic_cloud_clusterColored.push_back(myPoint);
                }

                if (height1 < 0.5 || length1 * width1 < 1)
                    continue;

                boundingBoxes.emplace_back(Eigen::Vector3f(center_point1.x, center_point1.y, center_point1.z),
                                           Eigen::Vector3f(length1, width1, height1),
                                           heading1);
            }
            continue;
        }

        if (height < 0.5)
            continue;

        boundingBoxes.emplace_back(Eigen::Vector3f(center_point.x, center_point.y, center_point.z),
                                   Eigen::Vector3f(length, width, height),
                                   heading);
    }
}