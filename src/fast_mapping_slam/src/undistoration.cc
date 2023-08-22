#include "../include/undistoration.h"
const int queueLength = 2000;


undistoration::undistoration()
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> lidar undistortion node started.\033[0m");
    first_scan_ = true;
    corrected_pointcloud_.reset(new PointCloudT());
    // sub_ = nh_.subscribe("/imu/data", 10, undistoration::ImuCallback, this);
    // 参数进行初始化与重置
    ResetParameters();
}


sensor_msgs::LaserScan::ConstPtr undistoration::CorrectLaserScan(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    //处理laser数据
    // 缓存雷达数据
    if (!CacheLaserScan(scan_msg))
        ROS_INFO_STREAM("cache laser");
        return scan_msg;
    //处理imu数据
    // 如果使用imu，就对imu的数据进行修剪，进行时间同步，并计算雷达数据时间内的旋转
    if (use_imu_)
    {
        ROS_INFO_STREAM("use imu");
        
        if (!PruneImuDeque())
            ROS_INFO_STREAM("cache imu");
            return scan_msg;
            
    }
    //进行laser数据去畸变
    sensor_msgs::LaserScan::ConstPtr new_scan = computedistoration();

    
    ResetParameters();

    return new_scan;
}



// 缓存雷达数据
bool undistoration::CacheLaserScan(const sensor_msgs::LaserScan::ConstPtr &laserScanMsg)
{   
    if (first_scan_)
    {
        first_scan_ = false;

        // 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
        CreateAngleCache(laserScanMsg);

        scan_count_ = laserScanMsg->ranges.size();
    }

    corrected_pointcloud_->points.resize(laserScanMsg->ranges.size());

    // 缓存雷达数据
    laser_queue_.push_back(*laserScanMsg);
    ROS_INFO_STREAM("pushback laser finish");
    // 缓存两帧雷达数据，以防止imu或者odom的数据不能包含雷达数据
    if (laser_queue_.size() < 2)
        return false;

    // 取出队列中的第一个数据
    current_laserscan_ = boost::make_shared<sensor_msgs::LaserScan>(laser_queue_.front());

    // current_laserscan_ = laser_queue_.front();
    laser_queue_.pop_front();

    // 获取这帧雷达数据的起始，结束时间
    current_laserscan_header_ = current_laserscan_->header;
    current_scan_time_start_ = current_laserscan_header_.stamp.toSec(); // 认为ros中header的时间为这一帧雷达数据的起始时间
    current_scan_time_increment_ = current_laserscan_->time_increment;
    current_scan_time_end_ = current_scan_time_start_ + current_scan_time_increment_ * (scan_count_ - 1);
    ROS_INFO_STREAM("laser data process collect");
    return true;
}


// 修剪imu队列，以获取包含 当前帧雷达时间 的imu数据及转角
bool undistoration::PruneImuDeque()
{
    // std::lock_guard<std::mutex> lock(imu_lock_);

    // imu数据队列的头尾的时间戳要在雷达数据的时间段外
    // 需要在主函数中使用imu_queue_
    if (imu_queue_.empty() ||
        imu_queue_.front().header.stamp.toSec() > current_scan_time_start_ ||
        imu_queue_.back().header.stamp.toSec() < current_scan_time_end_)
    {
        ROS_WARN("Waiting for IMU data ...");
        return false;
    }

    // 修剪imu的数据队列，直到imu的时间接近这帧点云的时间
    while (!imu_queue_.empty())
    {
        if (imu_queue_.front().header.stamp.toSec() < current_scan_time_start_ - 0.1)
            imu_queue_.pop_front();
        else
            break;
    }

    if (imu_queue_.empty())
        return false;

    current_imu_index_ = 0;

    sensor_msgs::Imu tmp_imu_msg;
    double current_imu_time, time_diff;

    for (int i = 0; i < (int)imu_queue_.size(); i++)
    {
        tmp_imu_msg = imu_queue_[i];
        current_imu_time = tmp_imu_msg.header.stamp.toSec();

        if (current_imu_time < current_scan_time_start_)
        {
            // 初始角度为0
            if (current_imu_index_ == 0)
            {
                imu_rot_x_[0] = 0;
                imu_rot_y_[0] = 0;
                imu_rot_z_[0] = 0;
                imu_time_[0] = current_imu_time;
                ++current_imu_index_;
            }
            continue;
        }

        // imu时间比雷达结束时间晚，就退出
        if (current_imu_time > current_scan_time_end_)
            break;

        // get angular velocity
        double angular_x, angular_y, angular_z;
        angular_x = tmp_imu_msg.angular_velocity.x;
        angular_y = tmp_imu_msg.angular_velocity.y;
        angular_z = tmp_imu_msg.angular_velocity.z;

        // 对imu的角速度进行积分，当前帧的角度 = 上一帧的角度 + 当前帧角速度 * (当前帧imu的时间 - 上一帧imu的时间)
        double time_diff = current_imu_time - imu_time_[current_imu_index_ - 1];
        imu_rot_x_[current_imu_index_] = imu_rot_x_[current_imu_index_ - 1] + angular_x * time_diff;
        imu_rot_y_[current_imu_index_] = imu_rot_y_[current_imu_index_ - 1] + angular_y * time_diff;
        imu_rot_z_[current_imu_index_] = imu_rot_z_[current_imu_index_ - 1] + angular_z * time_diff;
        imu_time_[current_imu_index_] = current_imu_time;
        ++current_imu_index_;
    }

    // 对current_imu_index_进行-1操作后，current_imu_index_指向当前雷达时间内的最后一个imu数据
    --current_imu_index_;

    return true;
}

// void undistoration::ImuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
// {
//     // std::lock_guard<std::mutex> lock(imu_lock_);
//     imu_queue_.push_back(*imuMsg);
// }

// 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
void undistoration::CreateAngleCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
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
}

sensor_msgs::LaserScan::ConstPtr undistoration::computedistoration()
{

    
    sensor_msgs::LaserScan::ConstPtr corrected_scan = current_laserscan_; // 复制原始扫描消息的元数据
    // 先把corrected_scan里的东西拷出来 修改 修改完了再放回去
    sensor_msgs::LaserScan scan_copy = *corrected_scan;
    
    
    bool first_point_flag = true;
    double current_point_time = 0;
    double current_point_x = 0, current_point_y = 0, current_point_z = 1.0;

    Eigen::Affine3f transStartInverse, transFinal, transBt;

    for (int i = 0; i < scan_count_; i++)
    {
        // 如果是无效点，就跳过
        if (!std::isfinite(current_laserscan_->ranges[i]) ||
            current_laserscan_->ranges[i] < current_laserscan_->range_min ||
            current_laserscan_->ranges[i] > current_laserscan_->range_max)
        {
            scan_copy.ranges[i] = std::numeric_limits<float>::quiet_NaN(); // 标记为无效值
            continue;
        }

        current_point_time = current_scan_time_start_ + i * current_scan_time_increment_;

        // 计算雷达数据的 x y 坐标
        current_point_x = current_laserscan_->ranges[i] * a_cos_[i];
        current_point_y = current_laserscan_->ranges[i] * a_sin_[i];

        float rotXCur = 0, rotYCur = 0, rotZCur = 0;
        float posXCur = 0, posYCur = 0, posZCur = 0;

        // 求得当前点对应时刻 相对于start_odom_time_ 的平移与旋转
        if (use_imu_)
            ComputeRotation(current_point_time, &rotXCur, &rotYCur, &rotZCur);


        // 雷达数据的第一个点对应时刻 相对于start_odom_time_ 的平移与旋转，之后在这帧数据畸变过程中不再改变
        if (first_point_flag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur,
                                                        rotXCur, rotYCur, rotZCur))
                                    .inverse();
            first_point_flag = false;
        }

        // 当前点对应时刻 相对于start_odom_time_ 的平移与旋转
        transFinal = pcl::getTransformation(posXCur, posYCur, posZCur,
                                            rotXCur, rotYCur, rotZCur);

        // 雷达数据的第一个点对应时刻的激光雷达坐标系 到 雷达数据当前点对应时刻的激光雷达坐标系 间的坐标变换
        transBt = transStartInverse * transFinal;

        // 将当前点的坐标 加上 两个时刻坐标系间的坐标变换 
        // 得到 当前点在 雷达数据的第一个点对应时刻的激光雷达坐标系 下的坐标
        double corrected_range = transBt(0, 0) * current_point_x + transBt(0, 1) * current_point_y + transBt(0, 2) * current_point_z + transBt(0, 3);
        scan_copy.ranges[i] = corrected_range;
    }
    corrected_scan = boost::make_shared<const sensor_msgs::LaserScan>(scan_copy);
    return corrected_scan;
}

// 根据点云中某点的时间戳赋予其 通过插值 得到的旋转量
void undistoration::ComputeRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0;
    *rotYCur = 0;
    *rotZCur = 0;

    // 找到在　pointTime　之后的imu数据
    int imuPointerFront = 0;
    while (imuPointerFront < current_imu_index_)
    {
        if (pointTime < imu_time_[imuPointerFront])
            break;
        ++imuPointerFront;
    }

    // 如果上边的循环没进去或者到了最大执行次数，则只能近似的将当前的旋转　赋值过去
    if (pointTime > imu_time_[imuPointerFront] || imuPointerFront == 0)
    {
        *rotXCur = imu_rot_x_[imuPointerFront];
        *rotYCur = imu_rot_y_[imuPointerFront];
        *rotZCur = imu_rot_z_[imuPointerFront];
    }
    else
    {
        int imuPointerBack = imuPointerFront - 1;

        // 根据线性插值计算 pointTime 时刻的旋转
        double ratioFront = (pointTime - imu_time_[imuPointerBack]) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);
        double ratioBack = (imu_time_[imuPointerFront] - pointTime) / (imu_time_[imuPointerFront] - imu_time_[imuPointerBack]);

        *rotXCur = imu_rot_x_[imuPointerFront] * ratioFront + imu_rot_x_[imuPointerBack] * ratioBack;
        *rotYCur = imu_rot_y_[imuPointerFront] * ratioFront + imu_rot_y_[imuPointerBack] * ratioBack;
        *rotZCur = imu_rot_z_[imuPointerFront] * ratioFront + imu_rot_z_[imuPointerBack] * ratioBack;
    }
}

void undistoration::ResetParameters()
{
    corrected_pointcloud_->points.clear();

    current_imu_index_ = 0;

    imu_time_.clear();
    imu_rot_x_.clear();
    imu_rot_y_.clear();
    imu_rot_z_.clear();

    imu_time_ = std::vector<double>(queueLength, 0);
    imu_rot_x_ = std::vector<double>(queueLength, 0);
    imu_rot_y_ = std::vector<double>(queueLength, 0);
    imu_rot_z_ = std::vector<double>(queueLength, 0);

    odom_incre_x_ = 0.0;
    odom_incre_y_ = 0.0;
    odom_incre_z_ = 0.0;
}