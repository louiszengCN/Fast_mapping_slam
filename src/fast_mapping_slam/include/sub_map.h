#ifndef SUBMAP_BACKEND
#define SUBMAP_BACKEND
#include <cmath>
#include <vector>
#include <chrono>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
// tf2
#undef min
#undef max

struct Point_sub {
    int x;
    int y;
};


class submap_creater
{
public:
submap_creater();

std::vector<Point_sub> Bresenham_sub(int x0, int y0, int x1, int y1);
// 接受雷达数据 建立局部子图 返回子图


nav_msgs::OccupancyGrid CreateSubMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg, double r_x, double r_y, double r_theta);


private:
//map param
double resolution_sub;
int map_width_sub;
int map_height_sub;
double map_origin_x_sub;
double map_origin_y_sub;
double occupied_threshold_sub;
double free_threshold_sub;
double angle_robot_sub;
double x_robot_sub;
double y_robot_sub;
int grid_x_sub;
int grid_y_sub;
int robot_location_x_sub;
int robot_location_y_sub;
double temp_angle_increment_sub;
double temp_angle_min_sub;
std::vector<std::vector<int>> grid_map_sub;
nav_msgs::OccupancyGrid map_msg_sub;


};



#endif 