#ifndef MATCHER_BACKEND
#define MATCHER_BACKEND
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <nav_msgs/OccupancyGrid.h>
// 扫描匹配给出一个精位姿
class Backend {
public:
    
    std::vector<double> Backend_match(const sensor_msgs::LaserScan::ConstPtr& scan_msg, nav_msgs::OccupancyGrid map_msg, 
                                        nav_msgs::OccupancyGrid curr_map_msg);
    //Backend();
private:

    //pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    bool is_map_available;

    
    


};
#endif 
