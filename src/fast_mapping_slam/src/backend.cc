#include "../include/backend.h"


std::vector<double> Backend::Backend_match(const sensor_msgs::LaserScan::ConstPtr& scan_msg, 
                nav_msgs::OccupancyGrid last_map_msg, nav_msgs::OccupancyGrid curr_map_msg) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr last_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int row = 0; row < last_map_msg.info.height; ++row) {
        for (int col = 0; col < last_map_msg.info.width; ++col) {
            int index = row * last_map_msg.info.width + col;
            if (last_map_msg.data[index] == 100) { // 如果是占用点
                pcl::PointXYZ point;
                point.x = col * last_map_msg.info.resolution;
                point.y = row * last_map_msg.info.resolution;
                point.z = 0.0;
                last_map_cloud->push_back(point);
            } else if (last_map_msg.data[index] == 0) { // 如果是空闲点
                pcl::PointXYZ point;
                point.x = col * last_map_msg.info.resolution;
                point.y = row * last_map_msg.info.resolution;
                point.z = 0.0;
                last_map_cloud->push_back(point);
            }
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
       for (int row = 0; row < curr_map_msg.info.height; ++row) {
        for (int col = 0; col < curr_map_msg.info.width; ++col) {
            int index = row * curr_map_msg.info.width + col;
            if (curr_map_msg.data[index] == 100) { // 如果是占用点
                pcl::PointXYZ point;
                point.x = col * curr_map_msg.info.resolution;
                point.y = row * curr_map_msg.info.resolution;
                point.z = 0.0;
                curr_map_cloud->push_back(point);
            } else if (curr_map_msg.data[index] == 0) { // 如果是空闲点
                pcl::PointXYZ point;
                point.x = col * curr_map_msg.info.resolution;
                point.y = row * curr_map_msg.info.resolution;
                point.z = 0.0;
                curr_map_cloud->push_back(point);
            }
        }
    }

    // Perform ICP (Iterative Closest Point) scan-to-map matching
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(curr_map_cloud);
    icp.setInputTarget(last_map_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned_scan_cloud);

    if (icp.hasConverged()) {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        //ROS_INFO_STREAM("ICP transformation matrix:\n" << transformation);
        // Eigen::Matrix4f inverse_transformation = transformation.inverse();
        //  // 应用逆变换矩阵到 x, y
        // double x_l = inverse_transformation(0, 3);
        // double y_l = inverse_transformation(1, 3);
    
        // // 计算在激光雷达坐标系下的旋转角度
        // double theta_l = atan2(transformation(1, 0), transformation(0, 0));
    
        // // Store the pose in a vector
        // std::vector<double> pose = {x_l, y_l, theta_l};
        // return pose;
        // // 这里的xyz在栅格地图坐标系下 要先进行转换
        double x = transformation(0, 3);
        double y = transformation(1, 3);
        double theta = atan2(transformation(1, 0), transformation(0, 0));
        double x_l = x * 0.05 - 22.5;
        double y_l = y * 0.05 - 22.5;

        // Store the pose in a vector
        std::vector<double> pose = {x, y, theta};
        return pose;
    } else {
        std::vector<double> pose = {0, 0, 0};
        return pose;
    }
}




