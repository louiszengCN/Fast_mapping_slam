#ifndef LASER_SCAN_PROCESSOR_H
#define LASER_SCAN_PROCESSOR_H

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>

class LaserScanProcessor {
public:
    LaserScanProcessor();

    sensor_msgs::LaserScan::ConstPtr processScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

private:
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer;
    int buffer_size;
    sensor_msgs::LaserScan::Ptr compensated_scan;

    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
};

#endif // LASER_SCAN_PROCESSOR_H
