#include "../include/backend.h"


std::vector<double> Backend::Backend_match(const sensor_msgs::LaserScan::ConstPtr& scan_msg, nav_msgs::OccupancyGrid map_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int row = 0; row < map_msg.info.height; ++row) {
        for (int col = 0; col < map_msg.info.width; ++col) {
            int index = row * map_msg.info.width + col;
            if (map_msg.data[index] == 100) { // 如果是占用点
                pcl::PointXYZ point;
                point.x = col * map_msg.info.resolution;
                point.y = row * map_msg.info.resolution;
                point.z = 0.0;
                map_cloud->push_back(point);
            } else if (map_msg.data[index] == 0) { // 如果是空闲点
                pcl::PointXYZ point;
                point.x = col * map_msg.info.resolution;
                point.y = row * map_msg.info.resolution;
                point.z = 0.0;
                map_cloud->push_back(point);
            }
        }
    }

    // Convert LaserScan to PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        if (std::isfinite(range) && range >= scan_msg->range_min && range <= scan_msg->range_max) {
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0.0;
            scan_cloud->push_back(point);
        }
    }

    // Perform ICP (Iterative Closest Point) scan-to-map matching
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(scan_cloud);
    icp.setInputTarget(map_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned_scan_cloud);

    if (icp.hasConverged()) {
        Eigen::Matrix4f transformation = icp.getFinalTransformation();
        //ROS_INFO_STREAM("ICP transformation matrix:\n" << transformation);

        double x = transformation(0, 3);
        double y = transformation(1, 3);
        double theta = atan2(transformation(1, 0), transformation(0, 0));

        // Store the pose in a vector
        std::vector<double> pose = {x, y, theta};
        return pose;
    } else {
        std::vector<double> pose = {0, 0, 0};
        return pose;
    }
}




