/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LESSON2_SCAN_MATCH_PLICP
#define LESSON2_SCAN_MATCH_PLICP

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
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// csm
#include <csm/csm_all.h>
#undef min
#undef max

struct Point {
    int x;
    int y;
};

class ScanMatchPLICP
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher odom_publisher_;         // 声明一个Publisher
    ros::Publisher path_publisher_;
    ros::Publisher map_publihser_;

    ros::Time last_icp_time_;
    ros::Time current_time_;

    geometry_msgs::Twist latest_velocity_;
    geometry_msgs::PoseStamped pose_stamped;
    nav_msgs::Path path_msg;
    nav_msgs::OccupancyGrid map_msg;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    // geometry_msgs::TransformStamped tf_msg_o_b_;
    geometry_msgs::TransformStamped tf_msg_m_o_;

    nav_msgs::Odometry odom_msg;

    tf2::Transform base_to_laser_;    
    tf2::Transform laser_to_base_; 
    tf2::Transform odom_to_map_;    
    tf2::Transform map_to_odom_;    


    tf2::Transform base_in_odom_;           // base_link在odom坐标系下的坐标
    tf2::Transform base_in_odom_keyframe_;  // base_link在odom坐标系下的keyframe的坐标
    tf2::Transform base_in_odom_bkd;           // base_link在odom坐标系下的坐标

    // parameters
    bool initialized_;

    std::string odom_frame_;
    std::string base_frame_;

    double kf_dist_linear_;
    double kf_dist_linear_sq_;
    double kf_dist_angular_;
    int kf_scan_count_;
    int scan_count_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    //map param
    double resolution;
    int map_width;
    int map_height;
    double map_origin_x;
    double map_origin_y;
    double occupied_threshold;
    double free_threshold;
    double angle_robot;
    double x_robot;
    double y_robot;
    int grid_x;
    int grid_y;
    int robot_location_x;
    int robot_location_y;
    double temp_angle_increment;
    double temp_angle_min;
    std::vector<std::vector<int>> grid_map;



    // csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    void InitParams();
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    bool GetBaseToLaserTf(const std::string &frame_id);
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
    void ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time);
    void GetPrediction(double &prediction_change_x, double &prediction_change_y, double &prediction_change_angle, double dt);
    void CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);
    void PublishTFAndOdometry();
    bool NewKeyframeNeeded(const tf2::Transform &d);
    void publish_path(const tf2::Transform &corr_ch);
    std::vector<Point> Bresenham(int x0, int y0, int x1, int y1);
   //void updateMap(const sensor_msgs::LaserScan::ConstPtr& scan_msg, double robot_x, double robot_y, double robot_theta);
    void publishMap();
public:
    ScanMatchPLICP();
    ~ScanMatchPLICP();
    
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

#endif // LESSON2_SCAN_MATCH_PLICP