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
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>
#include "../include/plicp_mapping.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
double robot_pose_x;
double robot_pose_y;
double robot_pose_theta;





ScanMatchPLICP::ScanMatchPLICP() : private_node_("~"), tf_listener_(tfBuffer_)
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> PLICP odometry started.\033[0m");
    // subscribe
    laser_scan_subscriber_ = node_handle_.subscribe(
        "/scan", 1, &ScanMatchPLICP::ScanCallback, this);
    
    // PATH_PARAMS
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "odom_plicp"; // 设置坐标系，根据需要进行更改
    pose_stamped.header = path_msg.header;

    // publish
    odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_plicp", 50);
    path_publisher_ = node_handle_.advertise<nav_msgs::Path>("path_topic", 10);
    map_publihser_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

    // 参数初始化
    InitParams();

    scan_count_ = 0;

    // 第一帧雷达还未到来
    initialized_ = false;

    base_in_odom_.setIdentity();
    base_in_odom_keyframe_.setIdentity();

    input_.laser[0] = 0.0;
    input_.laser[1] = 0.0;
    input_.laser[2] = 0.0;

    // Initialize output_ vectors as Null for error-checking
    output_.cov_x_m = 0;
    output_.dx_dy1_m = 0;
    output_.dx_dy2_m = 0;
    //publishMap();
}

ScanMatchPLICP::~ScanMatchPLICP()
{
}

/*
 * ros与csm的参数初始化
 */
void ScanMatchPLICP::InitParams()
{
    private_node_.param<std::string>("odom_frame", odom_frame_, "odom_plicp");
    private_node_.param<std::string>("base_frame", base_frame_, "base_link");
    // **** keyframe params: when to generate the keyframe scan
    // if either is set to 0, reduces to frame-to-frame matching
    private_node_.param<double>("kf_dist_linear", kf_dist_linear_, 0.1);
    private_node_.param<double>("kf_dist_angular", kf_dist_angular_, 5.0 * (M_PI / 180.0));
    kf_dist_linear_sq_ = kf_dist_linear_ * kf_dist_linear_;
    private_node_.param<int>("kf_scan_count", kf_scan_count_, 10);
    // map params
    resolution = 0.05;
    map_width = 900;
    map_height = 900;
    //map_origin_x _y 就是map在rviz的右上角坐标，rviz中默认一个格子是1，记得乘以0.05变换成栅格坐标
    map_origin_x = -22.5;//-450*0.05 -450是map的中心点
    map_origin_y = -22.5;
    occupied_threshold = 0.7;
    free_threshold = 0.3;
    grid_map = std::vector<std::vector<int>>(map_height, std::vector<int>(map_width, 50));
    // map params
    map_msg.header.frame_id = "map";
    map_msg.info.width = map_width;
    map_msg.info.height = map_height;
    map_msg.info.resolution = resolution;
    map_msg.info.origin.position.x = map_origin_x;
    map_msg.info.origin.position.y = map_origin_y;
    map_msg.info.origin.orientation.w = 1.0;
    
    // tf odom-baselink   ORIGINATE
    // tf_msg_o_b_.header.stamp = current_time_;
    // tf_msg_o_b_.header.frame_id = odom_frame_;
    // tf_msg_o_b_.child_frame_id = base_frame_;
    // tf map-odom orignate
    // tf_msg_m_o_.header.stamp = current_time_;
    // tf_msg_m_o_.header.frame_id = "map";
    // tf_msg_m_o_.child_frame_id = odom_frame_;
    // odom
    //odom_msg.header.stamp = current_time_;
    odom_msg.header.frame_id = "odom_plicp";
    odom_msg.child_frame_id = "base_link";



    //grid_map.push_back(std::Vectormap_height, std::vector<int>(map_width, 50));

    // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)

    // Maximum angular displacement between scans
    if (!private_node_.getParam("max_angular_correction_deg", input_.max_angular_correction_deg))
        input_.max_angular_correction_deg = 45.0;

    // Maximum translation between scans (m)
    if (!private_node_.getParam("max_linear_correction", input_.max_linear_correction))
        input_.max_linear_correction = 1.0;

    // Maximum ICP cycle iterations
    if (!private_node_.getParam("max_iterations", input_.max_iterations))
        input_.max_iterations = 10;

    // A threshold for stopping (m)
    if (!private_node_.getParam("epsilon_xy", input_.epsilon_xy))
        input_.epsilon_xy = 0.000001;

    // A threshold for stopping (rad)
    if (!private_node_.getParam("epsilon_theta", input_.epsilon_theta))
        input_.epsilon_theta = 0.000001;

    // Maximum distance for a correspondence to be valid
    if (!private_node_.getParam("max_correspondence_dist", input_.max_correspondence_dist))
        input_.max_correspondence_dist = 1.0;

    // Noise in the scan (m)
    if (!private_node_.getParam("sigma", input_.sigma))
        input_.sigma = 0.010;

    // Use smart tricks for finding correspondences.
    if (!private_node_.getParam("use_corr_tricks", input_.use_corr_tricks))
        input_.use_corr_tricks = 1;

    // Restart: Restart if error is over threshold
    if (!private_node_.getParam("restart", input_.restart))
        input_.restart = 0;

    // Restart: Threshold for restarting
    if (!private_node_.getParam("restart_threshold_mean_error", input_.restart_threshold_mean_error))
        input_.restart_threshold_mean_error = 0.01;

    // Restart: displacement for restarting. (m)
    if (!private_node_.getParam("restart_dt", input_.restart_dt))
        input_.restart_dt = 1.0;

    // Restart: displacement for restarting. (rad)
    if (!private_node_.getParam("restart_dtheta", input_.restart_dtheta))
        input_.restart_dtheta = 0.1;

    // Max distance for staying in the same clustering
    if (!private_node_.getParam("clustering_threshold", input_.clustering_threshold))
        input_.clustering_threshold = 0.25;

    // Number of neighbour rays used to estimate the orientation
    if (!private_node_.getParam("orientation_neighbourhood", input_.orientation_neighbourhood))
        input_.orientation_neighbourhood = 20;

    // If 0, it's vanilla ICP
    if (!private_node_.getParam("use_point_to_line_distance", input_.use_point_to_line_distance))
        input_.use_point_to_line_distance = 1;

    // Discard correspondences based on the angles
    if (!private_node_.getParam("do_alpha_test", input_.do_alpha_test))
        input_.do_alpha_test = 0;

    // Discard correspondences based on the angles - threshold angle, in degrees
    if (!private_node_.getParam("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
        input_.do_alpha_test_thresholdDeg = 20.0;

    // Percentage of correspondences to consider: if 0.9,
    // always discard the top 10% of correspondences with more error
    if (!private_node_.getParam("outliers_maxPerc", input_.outliers_maxPerc))
        input_.outliers_maxPerc = 0.90;

    // Parameters describing a simple adaptive algorithm for discarding.
    //  1) Order the errors.
    //  2) Choose the percentile according to outliers_adaptive_order.
    //     (if it is 0.7, get the 70% percentile)
    //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
    //     with the value of the error at the chosen percentile.
    //  4) Discard correspondences over the threshold.
    //  This is useful to be conservative; yet remove the biggest errors.
    if (!private_node_.getParam("outliers_adaptive_order", input_.outliers_adaptive_order))
        input_.outliers_adaptive_order = 0.7;

    if (!private_node_.getParam("outliers_adaptive_mult", input_.outliers_adaptive_mult))
        input_.outliers_adaptive_mult = 2.0;

    // If you already have a guess of the solution, you can compute the polar angle
    // of the points of one scan in the new position. If the polar angle is not a monotone
    // function of the readings index, it means that the surface is not visible in the
    // next position. If it is not visible, then we don't use it for matching.
    if (!private_node_.getParam("do_visibility_test", input_.do_visibility_test))
        input_.do_visibility_test = 0;

    // no two points in laser_sens can have the same corr.
    if (!private_node_.getParam("outliers_remove_doubles", input_.outliers_remove_doubles))
        input_.outliers_remove_doubles = 1;

    // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
    if (!private_node_.getParam("do_compute_covariance", input_.do_compute_covariance))
        input_.do_compute_covariance = 0;

    // Checks that find_correspondences_tricks gives the right answer
    if (!private_node_.getParam("debug_verify_tricks", input_.debug_verify_tricks))
        input_.debug_verify_tricks = 0;

    // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
    // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
    if (!private_node_.getParam("use_ml_weights", input_.use_ml_weights))
        input_.use_ml_weights = 0;

    // If 1, the field 'readings_sigma' in the second scan is used to weight the
    // correspondence by 1/sigma^2
    if (!private_node_.getParam("use_sigma_weights", input_.use_sigma_weights))
        input_.use_sigma_weights = 0;
}

/*
 * 回调函数 进行数据处理
 */
void ScanMatchPLICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // 如果是第一帧数据，首先进行初始化，先缓存一下cos与sin值
    // 将 prev_ldp_scan_,last_icp_time_ 初始化
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    current_time_ = scan_msg->header.stamp;
    if (!initialized_)
    {
        // caches the sin and cos of all angles
        CreateCache(scan_msg);

        // 获取机器人坐标系与雷达坐标系间的坐标变换
        if (!GetBaseToLaserTf(scan_msg->header.frame_id))
        {
            ROS_WARN("Skipping scan");
            return;
        }

        LaserScanToLDP(scan_msg, prev_ldp_scan_);
        last_icp_time_ = current_time_;
        initialized_ = true;
        return;
    }

    // step1 进行数据类型转换
    start_time_ = std::chrono::steady_clock::now();

    LDP curr_ldp_scan;
    LaserScanToLDP(scan_msg, curr_ldp_scan);

    //end_time_ = std::chrono::steady_clock::now();
    //time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "\n转换数据格式用时: " << time_used_.count() << " 秒。" << std::endl;

    // step2 使用PLICP计算雷达前后两帧间的坐标变换
    start_time_ = std::chrono::steady_clock::now();

    ScanMatchWithPLICP(curr_ldp_scan, current_time_);

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
     std::cout << "PL-ICP匹配用时: " << time_used_.count() << " 秒。" << std::endl;
    //ROS_INFO("CACULATE POSE SUCCESS");

    //建图部分  
    std::chrono::steady_clock::time_point start_time_map = std::chrono::steady_clock::now();

    temp_angle_increment = scan_msg->angle_increment;
    temp_angle_min = scan_msg->angle_min;
    std::vector<float> ranges = scan_msg->ranges;
    
    for (size_t i = 0; i < ranges.size(); ++i) {
        // 跳过无效点
        if(ranges[i]<scan_msg->range_min || ranges[i]>scan_msg->range_max)
        {
            continue;
        }
        angle_robot = temp_angle_min + i * temp_angle_increment;
        // 雷达hit点的坐标 laser坐标系下
        x_robot = ranges[i] * cos(angle_robot + robot_pose_theta) + robot_pose_x;
        y_robot = ranges[i] * sin(angle_robot + robot_pose_theta) + robot_pose_y;
        // // 雷达hit点坐标转换到栅格坐标系
        // grid_x = static_cast<int>((x_robot - map_origin_x) / resolution) + 450;
        // grid_y = static_cast<int>((y_robot - map_origin_y) / resolution) + 450;
        // // 找到机器人在地图中的位置 450是地图的中点坐标(900*900)
        // robot_location_x = static_cast<int>((robot_pose_x - map_origin_x) / resolution) + 450;
        // robot_location_y = static_cast<int>((robot_pose_y - map_origin_y) / resolution) + 450;
         
        // 这里不需要加偏移量，否则地图就不会在base_link脚下更新，会在偏移处更新，无法做到在机器人脚下建图
        // 雷达hit点坐标转换到栅格坐标系
        grid_x = static_cast<int>((x_robot - map_origin_x) / resolution) ;
        grid_y = static_cast<int>((y_robot - map_origin_y) / resolution) ;
        // 找到机器人在地图中的位置 450是地图的中点坐标(900*900)
        robot_location_x = static_cast<int>((robot_pose_x - map_origin_x) / resolution) ;
        robot_location_y = static_cast<int>((robot_pose_y - map_origin_y) / resolution) ;





        //grid_x grid_y是击中的坐标
        if (grid_x >= 0 && grid_x < map_width && grid_y >= 0 && grid_y < map_height) {
            //把击中的赋值100
            if(grid_map[grid_y][grid_x]<100)
            {
            grid_map[grid_y][grid_x] += 5;
            }
            else
            {
            grid_map[grid_y][grid_x] = 100;
            }
            // 没有击中的赋值0
            std::vector<Point> v1 = Bresenham(robot_location_x, robot_location_y, grid_x, grid_y);
            // V1是空闲的点
            for(int i = 0; i < v1.size(); i++)
            {
            if(grid_map[v1[i].y][v1[i].x]>0)
            {
            grid_map[v1[i].y][v1[i].x] += -1;
            }
            else
            {
            grid_map[v1[i].y][v1[i].x] = 0;
            }
            
            }
            
        }
    }
    publishMap();
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time_map);
     std::cout<<"建图用时: "<< time_used.count() << " 秒。" << std::endl;
    std::chrono::duration<double> whole_time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time_);
     std::cout<<"匹配用时: "<< whole_time_used.count() << " 秒。" << std::endl;
    std::chrono::duration<double> whole_time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
     std::cout<<"整体用时: "<< whole_time_used_.count() << " 秒。" << std::endl;


}

/**
 * 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
 */
void ScanMatchPLICP::CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }

    input_.min_reading = scan_msg->range_min;
    input_.max_reading = scan_msg->range_max;
}

/**
 * 获取机器人坐标系与雷达坐标系间的坐标变换
 */
bool ScanMatchPLICP::GetBaseToLaserTf(const std::string &frame_id)
{
    //laser-----base_link的静态变换
    //ros::Time t = ros::Time::now();
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = base_frame_; //base link
    transformStamped.child_frame_id = frame_id;  //laser
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;
    tf2::fromMsg(transformStamped.transform, base_to_laser_);
    laser_to_base_ = base_to_laser_.inverse();
    
    //odom------map 也是静态变换
    //ros::Time t = ros::Time::now();
    geometry_msgs::TransformStamped transformStamped_o_m;
    transformStamped_o_m.header.stamp = ros::Time::now();
    transformStamped_o_m.header.frame_id = "map"; //odom
    transformStamped_o_m.child_frame_id = odom_frame_;  //map
    transformStamped_o_m.transform.translation.x = 0.0;
    transformStamped_o_m.transform.translation.y = 0.0;
    transformStamped_o_m.transform.translation.z = 0.0;
    transformStamped_o_m.transform.rotation.x = 0.0;
    transformStamped_o_m.transform.rotation.y = 0.0;
    transformStamped_o_m.transform.rotation.z = 0.0;
    transformStamped_o_m.transform.rotation.w = 1.0;
    tf2::fromMsg(transformStamped_o_m.transform, map_to_odom_);
    odom_to_map_ = map_to_odom_.inverse();

    return true;



 
}

/**
 * 将雷达的数据格式转成 csm 需要的格式
 */
void ScanMatchPLICP::LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp)
{
    unsigned int n = scan_msg->ranges.size();
    ldp = ld_alloc_new(n);

    for (unsigned int i = 0; i < n; i++)
    {
        // calculate position in laser frame
        double r = scan_msg->ranges[i];

        if (r > scan_msg->range_min && r < scan_msg->range_max)
        {
            // fill in laser scan data
            ldp->valid[i] = 1;
            ldp->readings[i] = r;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1; // for invalid range
        }

        ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
        ldp->cluster[i] = -1;
    }

    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[n - 1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

void ScanMatchPLICP::publish_path(const tf2::Transform &corr_ch)
{
    //发布path
    // 不能只是简单的累加，这样子的轨迹会主要跑直线，转弯以后无法识别
    // 遍历output_中的位姿信息，将其添加到Path消息中

        // if(output_.x[0]<=0.01)
        // {
        // pose_stamped.pose.position.x = 0;
        // }
        // else{
        // pose_stamped.pose.position.x = corr_ch.getOrigin().getX();
        // }
        pose_stamped.pose.position.x = corr_ch.getOrigin().getX();
        pose_stamped.pose.position.y = corr_ch.getOrigin().getY();
        
        pose_stamped.pose.position.z = 0.0; // 如果没有高度信息，可以将其设置为0

        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = tf2::getYaw(corr_ch.getRotation());
        pose_stamped.pose.orientation.w = 1.0;

        path_msg.poses.push_back(pose_stamped);
        path_publisher_.publish(path_msg);

}


/**
 * 使用PLICP进行帧间位姿的计算
 */
void ScanMatchPLICP::ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time)
{
    // CSM is used in the following way:
    // The scans are always in the laser frame
    // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
    // The new scan (currLDPScan) has a pose equal to the movement
    // of the laser in the laser frame since the last scan
    // The computed correction is then propagated using the tf machinery

    prev_ldp_scan_->odometry[0] = 0.0;
    prev_ldp_scan_->odometry[1] = 0.0;
    prev_ldp_scan_->odometry[2] = 0.0;

    prev_ldp_scan_->estimate[0] = 0.0;
    prev_ldp_scan_->estimate[1] = 0.0;
    prev_ldp_scan_->estimate[2] = 0.0;

    prev_ldp_scan_->true_pose[0] = 0.0;
    prev_ldp_scan_->true_pose[1] = 0.0;
    prev_ldp_scan_->true_pose[2] = 0.0;

    input_.laser_ref = prev_ldp_scan_;
    input_.laser_sens = curr_ldp_scan;

    // 匀速模型，速度乘以时间，得到预测的odom坐标系下的位姿变换
    double dt = (time - last_icp_time_).toSec();
    double pr_ch_x, pr_ch_y, pr_ch_a;
    GetPrediction(pr_ch_x, pr_ch_y, pr_ch_a, dt);

    tf2::Transform prediction_change;
    CreateTfFromXYTheta(pr_ch_x, pr_ch_y, pr_ch_a, prediction_change);

    // account for the change since the last kf, in the fixed frame
    // 将odom坐标系下的坐标变换 转换成 base_in_odom_keyframe_坐标系下的坐标变换
    prediction_change = prediction_change * (base_in_odom_ * base_in_odom_keyframe_.inverse());

    // the predicted change of the laser's position, in the laser frame
    // 将base_link坐标系下的坐标变换 转换成 雷达坐标系下的坐标变换
    tf2::Transform prediction_change_lidar;
    prediction_change_lidar = laser_to_base_ * base_in_odom_.inverse() * prediction_change * base_in_odom_ * base_to_laser_;

    input_.first_guess[0] = prediction_change_lidar.getOrigin().getX();
    input_.first_guess[1] = prediction_change_lidar.getOrigin().getY();
    input_.first_guess[2] = tf2::getYaw(prediction_change_lidar.getRotation());

    // If they are non-Null, free covariance gsl matrices to avoid leaking memory
    if (output_.cov_x_m)
    {
        gsl_matrix_free(output_.cov_x_m);
        output_.cov_x_m = 0;
    }
    if (output_.dx_dy1_m)
    {
        gsl_matrix_free(output_.dx_dy1_m);
        output_.dx_dy1_m = 0;
    }
    if (output_.dx_dy2_m)
    {
        gsl_matrix_free(output_.dx_dy2_m);
        output_.dx_dy2_m = 0;
    }
    
    start_time_ = std::chrono::steady_clock::now();
    // 调用csm进行plicp计算
    sm_icp(&input_, &output_);
    //laser下的
    // 不能写下面的，因为这是在laser系下，如果这样写，在odom和map系下观察就会很怪
    // 应该把这个坐标转入世界系(就是map或者odom系)
    // robot_pose_x = output_.x[0];
    // robot_pose_y = output_.x[1];
    // robot_pose_theta = output_.x[2];
  

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    // std::cout << "PLICP计算用时: " << time_used_.count() << " 秒。" << std::endl;
    //publish_path(output_);
  
    
    tf2::Transform corr_ch;

    if (output_.valid)
    {
        // 雷达坐标系下的坐标变换
        tf2::Transform corr_ch_l;
        CreateTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

        // 将雷达坐标系下的坐标变换 转换成 base_link坐标系下的坐标变换
        corr_ch = base_to_laser_ * corr_ch_l * laser_to_base_;
        
   


        // 更新 base_link 在 odom 坐标系下 的坐标
        base_in_odom_ = base_in_odom_keyframe_ * corr_ch;

        //在odom下（机器人的坐标从laser系转换到世界系去（map）） 这一步很重要
        // 如果没有这一步 那么在map系下看地图就不是在base_link脚下更新的，同时由于坐标系的差距
        // 在栅格中更新的坐标位置也会有差距，导致建图发生重叠
        robot_pose_x = base_in_odom_.getOrigin().getX();
        robot_pose_y = base_in_odom_.getOrigin().getY();
        robot_pose_theta = base_in_odom_.getOrigin().getZ();



        publish_path(base_in_odom_);
        latest_velocity_.linear.x = corr_ch.getOrigin().getX() / dt;
        latest_velocity_.angular.z = tf2::getYaw(corr_ch.getRotation()) / dt;
    }
    else
    {
        ROS_WARN("not Converged");
    }

    // 发布tf与odom话题
    PublishTFAndOdometry();

    // 检查是否需要更新关键帧坐标
    if (NewKeyframeNeeded(corr_ch))
    {
        // 更新关键帧坐标
        ld_free(prev_ldp_scan_);
        prev_ldp_scan_ = curr_ldp_scan;
        base_in_odom_keyframe_ = base_in_odom_;
    }
    else
    {
        ld_free(curr_ldp_scan);
    }

    last_icp_time_ = time;
}

/**
 * 推测从上次icp的时间到当前时刻间的坐标变换
 * 使用匀速模型，根据当前的速度，乘以时间，得到推测出来的位移
 */
void ScanMatchPLICP::GetPrediction(double &prediction_change_x,
                                   double &prediction_change_y,
                                   double &prediction_change_angle,
                                   double dt)
{
    // 速度小于 1e-6 , 则认为是静止的
    prediction_change_x = latest_velocity_.linear.x < 1e-6 ? 0.0 : dt * latest_velocity_.linear.x;
    prediction_change_y = latest_velocity_.linear.y < 1e-6 ? 0.0 : dt * latest_velocity_.linear.y;
    prediction_change_angle = latest_velocity_.linear.z < 1e-6 ? 0.0 : dt * latest_velocity_.linear.z;

    if (prediction_change_angle >= M_PI)
        prediction_change_angle -= 2.0 * M_PI;
    else if (prediction_change_angle < -M_PI)
        prediction_change_angle += 2.0 * M_PI;
}

/**tf_msg
 * 从x,y,theta创建tf
 */
void ScanMatchPLICP::CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform &t)
{
    t.setOrigin(tf2::Vector3(x, y, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    t.setRotation(q);
}

/**
 * 发布tf与odom话题
 */
void ScanMatchPLICP::PublishTFAndOdometry()
{
    // 只发布一次就可以了，这些都不会变化的
    // 错了，动态的需要一直更新才行
    // ODOM----BASELINK
    //geometry_msgs::TransformStamped tf_msg;
    // tf_msg.header.stamp = current_time_;
    // tf_msg.header.frame_id = odom_frame_;
    // tf_msg.child_frame_id = base_frame_;
    geometry_msgs::TransformStamped tf_msg_o_b_;
    tf_msg_o_b_.header.stamp = ros::Time::now();
    tf_msg_o_b_.header.frame_id = odom_frame_;
    tf_msg_o_b_.child_frame_id = base_frame_;
    tf_msg_o_b_.transform = tf2::toMsg(base_in_odom_);
    // 发布 odom 到 base_link 的 tf
    tf_broadcaster_.sendTransform(tf_msg_o_b_);

    // 发布 base_link到laser的
    geometry_msgs::TransformStamped tf_msg_l_b_;
    tf_msg_l_b_.header.stamp = ros::Time::now();
    tf_msg_l_b_.header.frame_id = "base_link";
    tf_msg_l_b_.child_frame_id = "laser";
    tf_msg_l_b_.transform = tf2::toMsg(base_to_laser_);
    tf_broadcaster_.sendTransform(tf_msg_l_b_);




    //发布map到odom的
    tf_msg_m_o_.header.frame_id = "map";
    tf_msg_m_o_.header.stamp = ros::Time::now();
    tf_msg_m_o_.child_frame_id = odom_frame_;
    tf_msg_m_o_.transform = tf2::toMsg(map_to_odom_);
    tf_broadcaster_.sendTransform(tf_msg_m_o_);

    // MAP------ODOM

    //nav_msgs::Odometry odom_msg;
    // odom也是发布一次就可以了，不需要一直维护
    odom_msg.header.stamp = ros::Time::now();
    // odom_msg.header.frame_id = "odom";
    // odom_msg.child_frame_id = "base_link";
    tf2::toMsg(base_in_odom_, odom_msg.pose.pose);
    odom_msg.twist.twist = latest_velocity_;

    // 发布 odomemtry 话题
    odom_publisher_.publish(odom_msg);
}

/**
 * 如果平移大于阈值，角度大于阈值，则创新新的关键帧
 * @return 需要创建关键帧返回true, 否则返回false
 */
bool ScanMatchPLICP::NewKeyframeNeeded(const tf2::Transform &d)
{
    scan_count_++;

    if (fabs(tf2::getYaw(d.getRotation())) > kf_dist_angular_)
        return true;

    if (scan_count_ == kf_scan_count_)
    {
        scan_count_ = 0;
        return true;
    }
        
    double x = d.getOrigin().getX();
    double y = d.getOrigin().getY();
    if (x * x + y * y > kf_dist_linear_sq_)
        return true;

    return false;
}






std::vector<Point> ScanMatchPLICP::Bresenham(int x0, int y0, int x1, int y1) 
{
    std::vector<Point> points;

    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep) {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int deltaX = x1 - x0;
    int deltaY = std::abs(y1 - y0);
    int error = 0;
    int ystep;
    int y = y0;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (int x = x0; x < x1; x++) {
        int pointX, pointY;
        if (steep) {
            pointX = y;
            pointY = x;
        } else {
            pointX = x;
            pointY = y;
        }

        points.push_back({pointX, pointY});

        error += deltaY;

        if (2 * error >= deltaX) {
            y += ystep;
            error -= deltaX;
        }
    }

    return points;

}

void ScanMatchPLICP::publishMap()
{
        std::vector<int8_t> flat_map;
        for (const auto& row : grid_map) {
            for (int cell : row) {
                if(cell == 50)
                {
                flat_map.push_back(static_cast<int8_t>(-1));
                }
                else if(cell > 50)
                {
                flat_map.push_back(static_cast<int8_t>(100));
                //flat_map.push_back(static_cast<int8_t>(cell));

                }
                else if(cell < 50)
                {
                flat_map.push_back(static_cast<int8_t>(0));
                //flat_map.push_back(static_cast<int8_t>(cell));
                }
                //flat_map.push_back(static_cast<int8_t>(cell));
            }
        }
       
        map_msg.data = flat_map;
        map_msg.header.stamp = ros::Time::now();
        map_publihser_.publish(map_msg);

        std::cout<<"publish map"<<std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson3_scan_match_plicp_node"); // 节点的名字
    ScanMatchPLICP scan_match_plicp;
    //OccupancyGridMapper mapper;

    ros::spin();
    return 0;
}