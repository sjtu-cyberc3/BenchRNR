#include "../include/utilis.h"

bool getTrajectoryParams(const std::string& filepath, int traj_num,
    double& start_time, double& end_time, int& model_init, double& start_x, double& start_y)
{
    std::ifstream infile(filepath);
    if (!infile.is_open()) {
        std::cerr << "Cannot open file: " << filepath << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int traj;
        double start, end;
        int init;
        if (!(iss >> traj >> start >> end >> init >> start_x >> start_y)) {
            continue;  // skip malformed lines
        }

        if (traj == traj_num) {
            start_time = start;
            end_time = end;
            model_init = init;
            return true;
            }
    }

    std::cerr << "Trajectory number " << traj_num << " not found in file." << std::endl;
    return false;
}



pcl::PointCloud<pcl::PointXYZI> VoxelDownsample(const pcl::PointCloud<pcl::PointXYZI>::Ptr frame,
                                                double voxel_size)
{
    std::unordered_map<Voxel, pcl::PointXYZI, VoxelHash> grid;
    grid.reserve(frame->size());
    for (const auto &point_pcl : frame->points)
    {
        Eigen::Vector3d point(point_pcl.x, point_pcl.y, point_pcl.z);
        Voxel voxel = Voxel((point / voxel_size).cast<int>());
        if (grid.find(voxel) != grid.end())
            continue;
        grid.insert({voxel, point_pcl});
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr frame_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    frame_downsampled->points.resize(grid.size());

    int i = 0;
    for (const auto &[voxel, point] : grid)
    {
        (void)voxel;
        frame_downsampled->points[i] = point;
        i++;
    }
    return *frame_downsampled;// 
}

// Function: Extract point cloud within a bounding box
pcl::PointCloud<pcl::PointXYZI> extractPointsInsideBoundingBox(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud,
    const BoundingBox& bbox)
{
    pcl::PointCloud<pcl::PointXYZI> pointsInside;

    // Compute rotation matrix (rotation around the Z-axis)
    Eigen::Matrix2f rotationMatrix;
    rotationMatrix << std::cos(bbox.theta), -std::sin(bbox.theta),
                      std::sin(bbox.theta),  std::cos(bbox.theta);

    // Half size of the bounding box
    Eigen::Vector3f halfSize = bbox.size / 2.0;

    for (const auto& point : inputCloud->points) {
        // Extract point coordinates
        Eigen::Vector3f pointVec(point.x, point.y, point.z);

        // Transform the point into the bounding box coordinate frame
        Eigen::Vector3f localPoint = pointVec - bbox.position;
        Eigen::Vector2f rotatedPoint2D = rotationMatrix.transpose() * localPoint.head<2>();

        // Check whether the point lies inside the bounding box
        if (std::abs(rotatedPoint2D.x()) <= halfSize.y() &&
            std::abs(rotatedPoint2D.y()) <= halfSize.x() &&
            std::abs(localPoint.z()) <= halfSize.z()) 
        {
            pointsInside.points.push_back(point);
        }
    }

    pointsInside.width = pointsInside.points.size();
    pointsInside.height = 1;
    pointsInside.is_dense = true;

    return pointsInside;
}

pcl::PointCloud<pcl::PointXYZI> VoxelDownsample_super(const pcl::PointCloud<pcl::PointXYZI>::Ptr frame, pcl::PointCloud<pcl::PointXYZI>::Ptr current_frame_downsampled,
                                                      int my_frame_num, double voxel_size)
{
    std::unordered_map<Voxel, pcl::PointXYZI, VoxelHash> grid;
    grid.reserve(frame->size());
    for(int i = 0; i < frame->size(); i++) {
        pcl::PointXYZI point_pcl = frame->points[i];
        Eigen::Vector3d point(point_pcl.x, point_pcl.y, point_pcl.z);
        Voxel voxel = Voxel((point / voxel_size).cast<int>());
        if (grid.find(voxel) != grid.end())
            continue;
        if(i < my_frame_num) {
            current_frame_downsampled->points.push_back(point_pcl);
        }
        grid.insert({voxel, point_pcl});
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr frame_downsampled(new pcl::PointCloud<pcl::PointXYZI>);
    frame_downsampled->points.resize(grid.size());

    int i = 0;
    for (const auto &[voxel, point] : grid)
    {
        (void)voxel;
        frame_downsampled->points[i] = point;
        i++;
    }
    return *frame_downsampled;
}


bool dbscan(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in, std::vector<std::vector<int>> &clusters_index, const double &r, const int &size)
{
    if (!cloud_in->size())
        return false;
    // LOG()
    pcl::KdTreeFLANN<pcl::PointXYZI> tree;
    tree.setInputCloud(cloud_in);
    std::vector<bool> cloud_processed(cloud_in->size(), false);

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_processed[i])
        {
            continue;
        }

        std::vector<int> seed_queue;
        std::vector<int> indices_cloud;
        std::vector<float> dists_cloud;
        if (tree.radiusSearch(cloud_in->points[i], r, indices_cloud, dists_cloud) >= size)
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
            if (tree.radiusSearch(cloud_in->points[seed_queue[seed_index]], r, indices, dists) < size)
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
        clusters_index.push_back(seed_queue);
    }
    // std::cout << clusters_index.size() << std::endl;

    if (clusters_index.size())
        return true;
    else
        return false;
}

// Perform normal vector analysis after projecting the point cloud onto a 2D plane
Eigen::MatrixXd pca(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_in)
{
    int size = cloud_in->points.size();
    Eigen::MatrixXd pointset(size, 2);
    // Construct an (x, y) matrix from the point cloud
    for (int i = 0; i < size; i++)
    {
        pointset(i, 0) = cloud_in->points[i].x;
        pointset(i, 1) = cloud_in->points[i].y;
    }
   // Compute the mean of each column (i.e., x and y)
    Eigen::MatrixXd meanval = pointset.colwise().mean();
    Eigen::RowVectorXd meanvecRow = meanval;
    pointset.rowwise() -= meanvecRow;
    // Compute the covariance matrix
    Eigen::MatrixXd ptp;
    ptp = pointset.adjoint() * pointset;
    ptp = ptp.array() / (size - 1);

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(ptp);

    return eig.eigenvectors().transpose();
}

void simplebbox(pcl::PointCloud<pcl::PointXYZI> &class_point_cloud, Eigen::Vector3f &center_point, double &yaw, double &length, double &width, double &height) // 简单的包围盒计算函数
{
    // compute the PCA of the point cloud
    Eigen::MatrixXd obj_pca = pca(class_point_cloud.makeShared());

    double x = obj_pca(0, 0);
    double y = obj_pca(0, 1);

    yaw = atan2(y, x);

    float max_x = -10000, max_y = -10000, max_z = -10000;
    float min_x = 10000, min_y = 10000, min_z = 0;

    for (int j = 0; j < class_point_cloud.size(); j++)
    {

        float temp_x = class_point_cloud[j].x * obj_pca(0, 0) + class_point_cloud[j].y * obj_pca(0, 1);
        float temp_y = class_point_cloud[j].x * obj_pca(1, 0) + class_point_cloud[j].y * obj_pca(1, 1);
        float temp_z = class_point_cloud[j].z;

        max_x = max(max_x, temp_x);
        max_y = max(max_y, temp_y);
        max_z = max(max_z, temp_z);

        min_x = min(min_x, temp_x);
        min_y = min(min_y, temp_y);
    }

    length = abs(max_x - min_x);
    width = abs(max_y - min_y);
    height = abs(max_z - min_z);

    float center_x = (max_x + min_x) / 2;
    float center_y = (max_y + min_y) / 2;

    center_point(0) = center_x * obj_pca(0, 0) + center_y * obj_pca(1, 0);
    center_point(1) = center_x * obj_pca(0, 1) + center_y * obj_pca(1, 1);
    center_point(2) = height/2;
}


void drawGTBBox(visualization_msgs::MarkerArray &marker_array, double timestamp, BoundingBox &gtbbox, std::string traj_num_str) {
    // Read ground truth from a txt file

    // Read the ground truth file
    // File name
    std::string filename = std::string(ROOT_DIR) + "/data/gt/gt_gps_"+ traj_num_str +".txt";


    // Open the file
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "cannot open file: " << filename << std::endl;
    }

    // 2D array (dynamic) to store the data
    std::vector<std::vector<double>> data;

    // Read the file line by line
    std::string line;
    while (std::getline(file, line))
    {
        std::vector<double> row;
        std::istringstream iss(line);
        double value;

        // Read values from each line one by one
        while (iss >> value)
        {
            row.push_back(value);
        }

        // Add the current row to the 2D array
        data.push_back(row);
    }

    
    // Find the ground truth entry closest to the given timestamp
    // Initialize variable
    int closest_index = 0;

// Traverse the first column to find the value closest to timestamp (within 0.01)
    for (size_t i = 1; i < data.size(); ++i)
    {
        double diff = std::abs(data[i][0] - timestamp);
        if (diff < 0.01)
        {
            closest_index = i;
            break;
        }
    }

    double x = data[closest_index][1];
    double y = data[closest_index][2];


    double length = 4.602;
    double width = 1.90;
    double height = 1.645;

    double z = height/2;

    // Compute rotation matrix
    double cos_yaw = -data[closest_index][7];
    double sin_yaw = data[closest_index][6];

    double yaw = atan2(sin_yaw, cos_yaw);

    BoundingBox bbox(Eigen::Vector3f(x, y, z), Eigen::Vector3f(length, width, height), yaw);

    gtbbox = bbox;

    // Compute the coordinates of the 8 bounding box corners
    geometry_msgs::Point point0;
    point0.x = x + (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point0.y = y + (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point0.z = z - height / 2;

    geometry_msgs::Point point1;
    point1.x = x + (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point1.y = y + (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point1.z = z - height / 2;

    geometry_msgs::Point point2;
    point2.x = x - (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point2.y = y - (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point2.z = z - height / 2;

    geometry_msgs::Point point3;
    point3.x = x - (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point3.y = y - (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point3.z = z - height / 2;

    geometry_msgs::Point point4;
    point4.x = x + (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point4.y = y + (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point4.z = z + height / 2;

    geometry_msgs::Point point5;
    point5.x = x + (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point5.y = y + (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point5.z = z + height / 2;

    geometry_msgs::Point point6;
    point6.x = x - (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point6.y = y - (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point6.z = z + height / 2;

    geometry_msgs::Point point7;
    point7.x = x - (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point7.y = y - (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point7.z = z + height / 2;

    std::vector<geometry_msgs::Point> vertex{point0, point1, point2, point3, point4, point5, point6, point7};
    std::vector<std::vector<int>> connection{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5}, {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}};

    int gt_marker_id = 10000;

    for (int i = 0; i < 12; i++)
    {
        visualization_msgs::Marker line_list;
        line_list.id = gt_marker_id++;
        line_list.lifetime = ros::Duration(0.1);
        line_list.header.frame_id = "rslidar";
        line_list.ns = "boundingbox";
        line_list.header.stamp = ros::Time::now();
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;

        line_list.color.r = 0.196;
        line_list.color.g = 1;
        line_list.color.b = 0.196;
        line_list.color.a = 1.0;

        // line_list.color.r = 154/255.0;
        // line_list.color.g = 208/255.0;
        // line_list.color.b = 43/255.0;
        // line_list.color.a = 1.0;

        line_list.points.push_back(vertex[connection[i][0]]);
        line_list.points.push_back(vertex[connection[i][1]]);
        marker_array.markers.push_back(line_list);
    }
}

void DrawBasedOnbbox_linebox(Eigen::Vector3f pos, double theta, Eigen::Vector3f size, visualization_msgs::MarkerArray &marker_array, int color_id) {
    double x = pos(0);
    double y = pos(1);


    double length = size(0);
    double width = size(1);
    double height = size(2);

    double z = height/2;


    // 计算旋转矩阵
    double cos_yaw = cos(theta);
    double sin_yaw = sin(theta);

    // 计算八个顶点的坐标
    geometry_msgs::Point point0;
    point0.x = x + (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point0.y = y + (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point0.z = z - height / 2;

    geometry_msgs::Point point1;
    point1.x = x + (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point1.y = y + (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point1.z = z - height / 2;

    geometry_msgs::Point point2;
    point2.x = x - (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point2.y = y - (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point2.z = z - height / 2;

    geometry_msgs::Point point3;
    point3.x = x - (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point3.y = y - (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point3.z = z - height / 2;

    geometry_msgs::Point point4;
    point4.x = x + (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point4.y = y + (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point4.z = z + height / 2;

    geometry_msgs::Point point5;
    point5.x = x + (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point5.y = y + (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point5.z = z + height / 2;

    geometry_msgs::Point point6;
    point6.x = x - (length / 2) * cos_yaw + (width / 2) * sin_yaw;
    point6.y = y - (length / 2) * sin_yaw - (width / 2) * cos_yaw;
    point6.z = z + height / 2;

    geometry_msgs::Point point7;
    point7.x = x - (length / 2) * cos_yaw - (width / 2) * sin_yaw;
    point7.y = y - (length / 2) * sin_yaw + (width / 2) * cos_yaw;
    point7.z = z + height / 2;

    std::vector<geometry_msgs::Point> vertex{point0, point1, point2, point3, point4, point5, point6, point7};
    std::vector<std::vector<int>> connection{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5}, {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}};

    // static int gt_marker_id = 10000;

    // static int this_marker_id = 2560;
    int this_marker_id = 2560;


    for (int i = 0; i < 12; i++)
    {
        visualization_msgs::Marker line_list;
        line_list.id = this_marker_id++;
        line_list.lifetime = ros::Duration(1);
        line_list.header.frame_id = "rslidar";
        line_list.ns = "boundingbox";
        line_list.header.stamp = ros::Time::now();
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;

        line_list.color.r = 237/255.0;
        line_list.color.g = 125/255.0;
        line_list.color.b = 49/255.0;
        line_list.color.a = 1;

        if(color_id == 1) {
            // line_list.color.r = 0;
            // line_list.color.g = 176/255.0;
            // line_list.color.b = 240/255.0;
            line_list.color.r = 174/255.0;
            line_list.color.g = 129/255.0;
            line_list.color.b = 255/255.0;
        }

        line_list.points.push_back(vertex[connection[i][0]]);
        line_list.points.push_back(vertex[connection[i][1]]);
        marker_array.markers.push_back(line_list);
    }
}



void MyBoundingBox(pcl::PointCloud<pcl::PointXYZI> &class_point_cloud, pcl::PointXYZI &center_point, Eigen::MatrixXd &obj_pca, visualization_msgs::MarkerArray &marker_array, int &marker_id, double &length, double &width, double &height, ros::Time stamp_time, pcl::PointXYZI &point_zhou, int color_id, double bias, int flag_time)
{

    float max_x = -10000, max_y = -10000, max_z = -10000;
    float min_x = 10000, min_y = 10000, min_z = 10000; // 每个聚类类别的最大最小点

    // 遍历此分类，根据PCA分析的结果来进行包围盒最大最小点的计算
    for (int j = 0; j < class_point_cloud.size(); j++)
    {

        float temp_x = class_point_cloud[j].x * obj_pca(0, 0) + class_point_cloud[j].y * obj_pca(0, 1);
        float temp_y = class_point_cloud[j].x * obj_pca(1, 0) + class_point_cloud[j].y * obj_pca(1, 1);
        float temp_z = class_point_cloud[j].z;

        // 求类别内的最大最小点，不过这是在以目标物自身正交分解的
        max_x = max(max_x, temp_x);
        max_y = max(max_y, temp_y);
        max_z = max(max_z, temp_z);

        min_x = min(min_x, temp_x);
        min_y = min(min_y, temp_y);
        min_z = min(min_z, temp_z);
    }

    length = abs(max_x - min_x);
    width = abs(max_y - min_y);
    height = abs(max_z - min_z);

    if (length < width) {
    std::swap(length, width);
    }   


    // 计算中心点

    float center_x = (max_x + min_x) / 2;
    float center_y = (max_y + min_y) / 2;
    float center_z = (max_z + min_z) / 2;


    center_point.x = center_x * obj_pca(0, 0) + center_y * obj_pca(1, 0);
    center_point.y = center_x * obj_pca(0, 1) + center_y * obj_pca(1, 1);
    center_point.z = center_z;

    double z_class_1 = 0.0;
    double z_class_2 = 0.0;

    double class_1_pt_x = 0;
    double class_1_pt_y = 0;

    double class_2_pt_x = 0;
    double class_2_pt_y = 0;

    int num_1 = 0;
    int num_2 = 0;


    double m = - obj_pca(0,1) * center_point.x + obj_pca(0,0) * center_point.y;

    for (int j = 0; j < class_point_cloud.size(); j++) {
        double criteon = - obj_pca(0,1) * class_point_cloud[j].x + obj_pca(0,0) * class_point_cloud[j].y - m;

        if(criteon > 0) {
            // z_class_1 += class_point_cloud[j].z;
            if(class_point_cloud[j].z > 0.5 * (min_z + max_z)) {
                z_class_1 += 1;
            }

            num_1++;

            class_1_pt_x += class_point_cloud[j].x;
            class_1_pt_y += class_point_cloud[j].y;

        } else {
            if(class_point_cloud[j].z > 0.5 * (min_z + max_z)) {
                z_class_2 += 1;
            }
            num_2++;

            class_2_pt_x += class_point_cloud[j].x;
            class_2_pt_y += class_point_cloud[j].y;
        }
    }

    z_class_1 /= num_1;
    z_class_2 /= num_2;

    class_1_pt_x /= num_1;
    class_1_pt_y /= num_1;

    class_2_pt_x /= num_2;
    class_2_pt_y /= num_2;

    double p_x = class_1_pt_x - class_2_pt_x;
    double p_y = class_1_pt_y - class_2_pt_y;

    if(z_class_1 < z_class_2) {
        p_x = -p_x;
        p_y = -p_y;
    }

    double flag = 1;

    if(p_x * obj_pca(0,0) + p_y * obj_pca(0,1) > 0) {
        flag = -1; 
    }

    double axis_offset = bias;

    // std::cout << "axis offset: " << axis_offset << std::endl;

    // flag *= -1;

    point_zhou.x = center_point.x - axis_offset * obj_pca(0, 1) * flag;
    point_zhou.y = center_point.y + axis_offset * obj_pca(0, 0) * flag;

    // // 3月16日-合创车GPS有一个0.5米的横向偏差 3月17日：在gps端补偿了
    // point_zhou.x -= 0.5 * obj_pca(0, 0) * flag;
    // point_zhou.y -= 0.5 * obj_pca(0, 1) * flag;

    point_zhou.z = (min_z + max_z) / 2;
    point_zhou.intensity = 100.0;

    min_z = 0;

    if(length * width > 20)
        return; // 如果计算得到的包围盒大小大于常规车辆，估计就是两个车被聚类成了一个，这里直接返回


    // 计算八个顶点的坐标
    geometry_msgs::Point point0;
    point0.x = obj_pca(0, 0) * min_x + obj_pca(1, 0) * min_y;
    point0.y = obj_pca(0, 1) * min_x + obj_pca(1, 1) * min_y;
    point0.z = min_z;
    geometry_msgs::Point point1;
    point1.x = obj_pca(0, 0) * min_x + obj_pca(1, 0) * max_y;
    point1.y = obj_pca(0, 1) * min_x + obj_pca(1, 1) * max_y;
    point1.z = min_z;
    geometry_msgs::Point point2;
    point2.x = obj_pca(0, 0) * max_x + obj_pca(1, 0) * max_y;
    point2.y = obj_pca(0, 1) * max_x + obj_pca(1, 1) * max_y;
    point2.z = min_z;
    geometry_msgs::Point point3;
    point3.x = obj_pca(0, 0) * max_x + obj_pca(1, 0) * min_y;
    point3.y = obj_pca(0, 1) * max_x + obj_pca(1, 1) * min_y;
    point3.z = min_z;
    geometry_msgs::Point point4;
    point4.x = obj_pca(0, 0) * min_x + obj_pca(1, 0) * min_y;
    point4.y = obj_pca(0, 1) * min_x + obj_pca(1, 1) * min_y;
    point4.z = max_z;
    geometry_msgs::Point point5;
    point5.x = obj_pca(0, 0) * min_x + obj_pca(1, 0) * max_y;
    point5.y = obj_pca(0, 1) * min_x + obj_pca(1, 1) * max_y;
    point5.z = max_z;
    geometry_msgs::Point point6;
    point6.x = obj_pca(0, 0) * max_x + obj_pca(1, 0) * max_y;
    point6.y = obj_pca(0, 1) * max_x + obj_pca(1, 1) * max_y;
    point6.z = max_z;
    geometry_msgs::Point point7;
    point7.x = obj_pca(0, 0) * max_x + obj_pca(1, 0) * min_y;
    point7.y = obj_pca(0, 1) * max_x + obj_pca(1, 1) * min_y;
    point7.z = max_z;

    vector<geometry_msgs::Point> vertex{point0, point1, point2, point3, point4, point5, point6, point7};
    vector<vector<int>> connection{{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5}, {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}};
    

    // 在输出前改变obj_pca的值，为后续输出到文件，优化axis_offset作准备
    double temp1 = - obj_pca(0, 1) * flag;
    double temp2 = obj_pca(0, 0) * flag;

    obj_pca(0, 0) = temp1;
    obj_pca(0, 1) = temp2;

    int loc_marker_id = 1999;

    for (int i = 0; i < 12; i++)
    {
        visualization_msgs::Marker line_list;
        line_list.id = marker_id;

        if (color_id == 2 || color_id == 3) {
            line_list.id = loc_marker_id++;  
        } else if(color_id == 1) {
            line_list.id = 2999 + i;
        }
        line_list.lifetime = ros::Duration(0.1);
        line_list.header.frame_id = "rslidar";
        line_list.ns = "boundingbox";
        line_list.header.stamp = stamp_time;
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;
        if (color_id == 0) {          // draw object detection result
            line_list.color.r = 1.0;
            line_list.color.g = 0.0;
            line_list.color.b = 0.0;
            line_list.color.a = 1.0;
            // return;
        }
        else if (color_id == 1) {    // draw register_loc result
            line_list.color.r = 100/255.0;
            line_list.color.g = 100/255.0;
            line_list.color.b = 255/255.0;
            line_list.color.a = 1.0;
        }
        else if (color_id == 2) {
            line_list.color.r = 0.44;
            line_list.color.g = 0.44;
            line_list.color.b = 1;
            line_list.color.a = 1.0;
        } else if(color_id == 3){
            line_list.color.r = 223/255.0;
            line_list.color.g = 147/255.0;
            line_list.color.b = 201/255.0;
            line_list.color.a = 1.0;
        }

        line_list.points.push_back(vertex[connection[i][0]]);
        line_list.points.push_back(vertex[connection[i][1]]);
        marker_array.markers.push_back(line_list);

        // std::cout << "marker_array: " << " " << marker_array.markers.size() << " " << line_list.id << std::endl;

        marker_id++;
    }
}


visualization_msgs::Marker DrawBasedOnbbox(Eigen::Vector3f pos, double theta, Eigen::Vector3f size, int marker_id, int color_id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "rslidar";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos(0);
    marker.pose.position.y = pos(1);
    marker.pose.position.z = pos(2);


    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sin(theta/2);
    marker.pose.orientation.w = cos(theta/2);

    marker.scale.x = size(0);
    marker.scale.y = size(1);
    marker.scale.z = size(2);
    marker.color.a = 0.5;

    if(color_id == 0) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    } else{
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
    }
    return marker;
}

Eigen::Vector2d calculateMarkerOrientation(const visualization_msgs::Marker& marker) {
    Eigen::Vector2d orientation;

    pcl::PointCloud<pcl::PointXYZI> points;
    for (const auto& point : marker.points) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        points.push_back(pcl_point);
    }

    std::cout << points.size() << std::endl;


    Eigen::MatrixXd pca_p = pca(points.makeShared());
    orientation << pca_p(0, 0), pca_p(0, 1);


    return orientation;
}


geometry_msgs::Point calculateMarkerCenter(const visualization_msgs::Marker& marker) {
    geometry_msgs::Point center;
    int numPoints = marker.points.size();

    if (numPoints > 0) {
        for (const auto& point : marker.points) {
            center.x += point.x;
            center.y += point.y;
            center.z += point.z;
        }

        center.x /= numPoints;
        center.y /= numPoints;
        center.z /= numPoints;
    }

    return center;
}

void getEntryVertices(const pcl::PointXYZI &entry_center, const double &entry_length, const double &entry_width,
                      const double &entry_rot_angle, std::vector<pcl::PointXYZI> &vertices){
    double entry_rot_radians = entry_rot_angle * M_PI / 180.0;                    

    vertices[0].x = entry_center.x - 0.5 * entry_length;
    vertices[0].y = entry_center.y - 0.5 * entry_width;
    vertices[1].x = entry_center.x + 0.5 * entry_length;
    vertices[1].y = entry_center.y - 0.5 * entry_width;
    vertices[2].x = entry_center.x + 0.5 * entry_length;
    vertices[2].y = entry_center.y + 0.5 * entry_width;
    vertices[3].x = entry_center.x - 0.5 * entry_length;
    vertices[3].y = entry_center.y + 0.5 * entry_width;

    for (auto &vertex : vertices) {
        double x_new = entry_center.x + (vertex.x - entry_center.x) * std::cos(entry_rot_radians) - (vertex.y - entry_center.y) * std::sin(entry_rot_radians);
        double y_new = entry_center.y + (vertex.x - entry_center.x) * std::sin(entry_rot_radians) + (vertex.y - entry_center.y) * std::cos(entry_rot_radians);
        vertex.x = x_new;
        vertex.y = y_new;
    }                
}

bool isPointInRegion(const pcl::PointXYZI &point, std::vector<pcl::PointXYZI> vertices) {
    int intersections = 0;
    for (int i = 0; i < 4; i++) {
        pcl::PointXYZI &v1 = vertices[i];
        pcl::PointXYZI &v2 = vertices[(i + 1) % 4];

        if (((v1.y > point.y) != (v2.y > point.y)) &&
            (point.x < (v2.x - v1.x) * (point.y - v1.y) / (v2.y - v1.y) + v1.x)) {
            intersections++;
        }
    }
    return (intersections % 2) == 1;
}

bool isPointCloudInRegion(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                          std::vector<pcl::PointXYZI> vertices) {
    int num_in_region = 0;
    for (const auto &point : cloud->points) {
        if (isPointInRegion(point,vertices)) {
            num_in_region++; 
        }
    }
    
    return (num_in_region > cloud->size() * 0.8) ? true : false;
}

void create_entry_box(visualization_msgs::MarkerArray &marker_array_entry, pcl::PointXYZI &entry_center,
                      double &entry_length, double &entry_width, double &entry_rot_angle, ros::Time stamp_time) {
    double entry_rot_radians = entry_rot_angle * M_PI / 180.0;
    
    geometry_msgs::Point point0;
    point0.x = entry_center.x - 0.5 * entry_length;
    point0.y = entry_center.y - 0.5 * entry_width;
    point0.z = 0;
    geometry_msgs::Point point1;
    point1.x = entry_center.x + 0.5 * entry_length;
    point1.y = entry_center.y - 0.5 * entry_width;
    point1.z = 0;
    geometry_msgs::Point point2;
    point2.x = entry_center.x + 0.5 * entry_length;
    point2.y = entry_center.y + 0.5 * entry_width;
    point2.z = 0;
    geometry_msgs::Point point3;
    point3.x = entry_center.x - 0.5 * entry_length;
    point3.y = entry_center.y + 0.5 * entry_width;
    point3.z = 0;

    std::vector<geometry_msgs::Point> vertices(4);
    for (int i = 0; i < 4; i++) {
        double x = (i == 0 || i == 3) ? point0.x : point1.x;
        double y = (i == 0 || i == 1) ? point0.y : point2.y;

        vertices[i].x = entry_center.x + (x - entry_center.x) * std::cos(entry_rot_radians) - (y - entry_center.y) * std::sin(entry_rot_radians);
        vertices[i].y = entry_center.y + (x - entry_center.x) * std::sin(entry_rot_radians) + (y - entry_center.y) * std::cos(entry_rot_radians);
        vertices[i].z = 0;
    }

    std::vector<std::vector<int>> connection{{0, 1}, {1, 2}, {2, 3}, {3, 0}};
    for (int i = 0; i < 4; i++) {
        visualization_msgs::Marker line_list;
        line_list.id = 5000 + i;
        line_list.lifetime = ros::Duration(0.1);
        line_list.header.frame_id = "rslidar";
        line_list.header.stamp = stamp_time;
        line_list.ns = "entry";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.1;
        line_list.color.r = 0.0;
        line_list.color.g = 0.0;
        line_list.color.b = 1.0;
        line_list.color.a = 1.0;
        line_list.points.push_back(vertices[connection[i][0]]);
        line_list.points.push_back(vertices[connection[i][1]]);
        marker_array_entry.markers.push_back(line_list);
    }
}