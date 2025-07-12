#include "../include/ceres_icp.h"

void my_icp(pcl::PointCloud<pcl::PointXYZI>& source, pcl::PointCloud<pcl::PointXYZI>& target, Eigen::Matrix3d& tf_matrix, bool transform_template)
{
    // -----------------加载点云数据---------------------------
	// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::io::loadPCDFile<pcl::PointXYZI>("/home/ycl/e100_cloud/car_3.pcd", *cloud1);
	// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::io::loadPCDFile<pcl::PointXYZI>("/home/ycl/e100_cloud/car_2.pcd", *cloud2);

    // pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);

    // //存儲最鄰近查詢後的對應點

    // //統計點個數，點數少的作爲源域點雲
    // if(cloud1->points.size() < cloud2->points.size()){
    //     *source = *cloud1;
    //     *target = *cloud2;
    // }else{
    //     *source = *cloud2;
    //     *target = *cloud1;
    // }

    // //求出目標域點雲的中心點
    // pcl::PointXYZ centroid_point;
    // centroid_point.x = 0;
    // centroid_point.y = 0;
    // centroid_point.z = 0;

    // for(int j = 0; j < target->points.size(); j++){
    //     centroid_point.x += target->points[j].x;
    //     centroid_point.y += target->points[j].y;
    //     centroid_point.z += target->points[j].z;
    // }

    // centroid_point.x = centroid_point.x/target->points.size();
    // centroid_point.y = centroid_point.y/target->points.size();
    // centroid_point.z = centroid_point.z/target->points.size();

    // for(int j = 0; j < target->points.size(); j++){
    //     target->points[j].x -= centroid_point.x;
    //     target->points[j].y -= centroid_point.y;
    //     target->points[j].z -= centroid_point.z;
    // }

    // for(int j = 0; j < source->points.size(); j++){
    //     source->points[j].x -= centroid_point.x;
    //     source->points[j].y -= centroid_point.y;
    //     source->points[j].z -= centroid_point.z;
    // }

    //KDTREE設定初始化
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_icp;
    kdtree_icp.setInputCloud(target.makeShared());

    int K = 1;

    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquareDistance(K);

    Eigen::Matrix3d final_tr_matrix;
    final_tr_matrix << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    int ceres_iter_num = 30;
    for(int j = 0; j < ceres_iter_num; j++){

        pcl::PointCloud<pcl::PointXYZI>::Ptr index(new pcl::PointCloud<pcl::PointXYZI>);
        //尋找最鄰近點
        for(int i = 0; i < source.points.size(); i++){
            if(kdtree_icp.nearestKSearch(source.points[i], K, pointIdxKNNSearch, pointKNNSquareDistance) > 0){
                //std::cout<<"Point in source point cloud is x = "<<source->points[i].x<<" y = "<<source->points[i].y<<" z = "<<source->points[i].z<<std::endl;
                //std::cout<<"Nearest point in target cloud is x = "<<target->points[pointIdxKNNSearch[0]].x<<" y = "<<target->points[pointIdxKNNSearch[0]].y<<" z = "<<target->points[pointIdxKNNSearch[0]].z<<std::endl;
                //std::cout<<"Nearest distance is: "<<pointKNNSquareDistance[0]<<std::endl;
            }else{
                //std::cout<<"No found nearest point"<<std::endl;
            }
            index->points.push_back(target.points[pointIdxKNNSearch[0]]);
        }

        //std::cout<<"Source point cloud num is: "<<source->points.size()<<std::endl;
        //std::cout<<"Index point cloud num is: "<<index->points.size()<<std::endl;

        index->height = 1;
        index->width = index->points.size();

        //pcl::io::savePCDFile<pcl::PointXYZI>("/home/ycl/e100_cloud/car_3_nearest.pcd", *index);

        double residual_priv = 0;

        for(int i = 0; i < source.points.size(); i++){
            double dx = source.points[i].x - index->points[i].x;
            double dy = source.points[i].y - index->points[i].y;
            residual_priv += dx*dx + dy*dy;
        }

        //待優化變量 角度和平移向量
        double yaw_arr = 0;
        double t_arr[2] = {0, 0};

        ceres::Problem problem;
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        //options.minimizer_progress_to_stdout = false;

        options.max_num_iterations = 50;
        options.num_threads = 4;

        ceres::Solver::Summary summary;
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);

        // ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

        // problem.AddParameterBlock(&yaw_arr, 1, angle_local_parameterization);
        problem.AddParameterBlock(&yaw_arr, 1);
        problem.AddParameterBlock(t_arr, 2);

        for(int i = 0; i < source.points.size(); i++){
            //Eigen::DiagonalMatrix<double, 2> conv(0.1, 0.1);
            Eigen::DiagonalMatrix<double, 2> conv(1, 1);
            Eigen::Matrix2d sqrt_information = conv;
            //sqrt_information = sqrt_information.inverse().llt().matrixL();
            double weight = 1.0;
            ceres::CostFunction* cost_function = PP2dErrorTerm::Create(source.points[i].x, source.points[i].y, index->points[i].x, index->points[i].y,
                                                                    sqrt_information, weight);
            //problem.AddResidualBlock(cost_function, loss_function, &yaw_arr, t_arr);
            problem.AddResidualBlock(cost_function, NULL, &yaw_arr, t_arr);
        }

        ceres::Solve(options, &problem, &summary);


        //根據優化結果進行對應變換
        for(int i = 0; i < source.points.size(); i++){
            source.points[i].x  = cos(yaw_arr) * source.points[i].x + -sin(yaw_arr) * source.points[i].y + t_arr[0];
            source.points[i].y  = sin(yaw_arr) * source.points[i].x + cos(yaw_arr) * source.points[i].y + t_arr[1];
        }

        double residual_new = 0;

        for(int i = 0; i < source.points.size(); i++){
            double dx = source.points[i].x - index->points[i].x;
            double dy = source.points[i].y - index->points[i].y;
            residual_new += dx*dx + dy*dy;
        }

        // if(j % 20 == 0) {
        //     std::cout << "Iteration: " << j << ", Residual before optimization: " << residual_priv << ", Residual after optimization: " << residual_new << " " << residual_new/source.points.size() << std::endl;
        // }

        Eigen::Matrix3d tr_matrix;
        tr_matrix << cos(yaw_arr), -sin(yaw_arr), t_arr[0], sin(yaw_arr), cos(yaw_arr), t_arr[1], 0, 0, 1;

        final_tr_matrix = tr_matrix * final_tr_matrix;
    }

    if(transform_template){
        //用得到的变换把模板点云变换到对应位置
        for(int i = 0; i < target.size(); i++){
            Eigen::Vector3d cur_vec;
            cur_vec << target[i].x, target[i].y, 1;
            cur_vec = final_tr_matrix.inverse() * cur_vec;
            target[i].x = cur_vec(0);
            target[i].y = cur_vec(1);
        }
    }

    tf_matrix =  tf_matrix * final_tr_matrix;
}
