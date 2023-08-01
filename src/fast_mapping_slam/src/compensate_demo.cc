#include <ros/ros.h>
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
    LaserScanProcessor() : nh("~") {
        scan_sub = nh.subscribe("/scan", 1, &LaserScanProcessor::scanCallback, this);
        // 初始化缓冲区大小
        buffer_size = 50;
        pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud_compensate", 10);
        pub_laser = nh.advertise<sensor_msgs::LaserScan>("compensate_scan", 10);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        // 将LaserScan消息转换为PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            if (scan_msg->ranges[i] < scan_msg->range_max && scan_msg->ranges[i] > scan_msg->range_min) {
                pcl::PointXYZ point;
                point.x = scan_msg->ranges[i] * cos(scan_msg->angle_min + i * scan_msg->angle_increment);
                point.y = scan_msg->ranges[i] * sin(scan_msg->angle_min + i * scan_msg->angle_increment);
                point.z = 0.0;
                cloud_in->push_back(point);
            }
        }

        // 将当前帧数据加入历史缓冲区
        buffer.push_back(cloud_in);

        // 当缓冲区大小达到指定值时，开始进行运动畸变去除
        if (buffer.size() >= buffer_size) {
            // 使用ICP算法进行运动畸变去除
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(buffer.front());
            icp.setInputTarget(buffer.back());
            pcl::PointCloud<pcl::PointXYZ> cloud_out;
            icp.align(cloud_out);

            // 清空历史缓冲区
            buffer.clear();

            // 将去畸变后的激光雷达数据进行旋转补偿
            sensor_msgs::LaserScan compensated_scan;
            compensated_scan.header = scan_msg->header;
            compensated_scan.angle_min = scan_msg->angle_min;
            compensated_scan.angle_max = scan_msg->angle_max;
            compensated_scan.angle_increment = scan_msg->angle_increment;
            compensated_scan.time_increment = scan_msg->time_increment;
            compensated_scan.scan_time = scan_msg->scan_time;
            compensated_scan.range_min = scan_msg->range_min;
            compensated_scan.range_max = scan_msg->range_max;
            compensated_scan.ranges.resize(scan_msg->ranges.size());

            // 计算旋转角度（假设机器人绕z轴旋转）
            // 获取旋转矩阵
            Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();

            // 将旋转矩阵转换为四元数表示
            Eigen::Quaternionf q(transformation_matrix.block<3, 3>(0, 0));

            // 计算绕Z轴的旋转角度
            double rotation_angle = 2.0 * atan2(q.z(), q.w());

            // 对每个激光点进行旋转补偿
            for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
                double original_angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                double compensated_angle = original_angle + rotation_angle;
                int compensated_index = (compensated_angle - compensated_scan.angle_min) / compensated_scan.angle_increment;
                if (compensated_index >= 0 && compensated_index < static_cast<int>(scan_msg->ranges.size())) {
                    compensated_scan.ranges[i] = scan_msg->ranges[compensated_index];
                }
            }

            // 发布补偿后的激光雷达数据
            pub_laser.publish(compensated_scan);

            // 将去畸变后的点云数据可视化
            visualizePointCloud(cloud_out);
        }
    }

    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
        // 可视化代码（根据你使用的可视化工具进行相应的可视化操作）
        // 例如，使用ROS中的rviz来可视化激光雷达数据
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(cloud, ros_cloud);
        ros_cloud.header.frame_id = "laser"; // 设置坐标系
        pub.publish(ros_cloud);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Publisher pub;
    ros::Publisher pub_laser;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer;
    int buffer_size;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_scan_processor");
    LaserScanProcessor processor;
    ros::spin();
    return 0;
}
