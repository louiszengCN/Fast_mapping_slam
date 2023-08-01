#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gmapping/gmapping.h>  // 使用gmapping的前端头文件

class ScanMatchingNode
{
public:
    ScanMatchingNode() : nh("~")
    {
        // 设置ROS参数
        nh.param<std::string>("scan_topic", scanTopic, "/scan");

        // 订阅激光雷达数据
        scanSubscriber = nh.subscribe(scanTopic, 1, &ScanMatchingNode::scanCallback, this);

        // 创建GMapping前端对象
        slam = new GMapping::SlamGMapping();

        // 初始化匹配结果
        lastPose.setZero();
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scanMsg)
    {
        // 将激光数据转换为Eigen格式
        Eigen::VectorXf ranges = Eigen::VectorXf::Map(scanMsg->ranges.data(), scanMsg->ranges.size());

        // 进行激光雷达扫描匹配
        Eigen::Vector3f poseEstimate;
        scanMatch(ranges, poseEstimate);

        // 输出匹配结果
        ROS_INFO("Estimated Pose: x=%f, y=%f, theta=%f", poseEstimate[0], poseEstimate[1], poseEstimate[2]);
    }

    void scanMatch(const Eigen::VectorXf &ranges, Eigen::Vector3f &poseEstimate)
    {
        // 使用GMapping前端的函数进行扫描匹配
        GMapping::OdometryReading odometryReading(0, 0, 0);
        GMapping::RangeReading rangeReading(ranges.size());
        rangeReading.setPose(GMapping::OrientedPoint(0, 0, 0));

        for (size_t i = 0; i < ranges.size(); ++i)
        {
            rangeReading[i] = ranges[i];
        }

        slam->scanMatch(rangeReading, odometryReading);

        // 获取匹配结果
        GMapping::OrientedPoint matchedPose = slam->getPose();

        // 转换为Eigen格式
        poseEstimate[0] = matchedPose.x;
        poseEstimate[1] = matchedPose.y;
        poseEstimate[2] = matchedPose.theta;
    }

    ~ScanMatchingNode()
    {
        if (slam)
        {
            delete slam;
            slam = nullptr;
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber scanSubscriber;

    std::string scanTopic;

    GMapping::SlamGMapping *slam;

    Eigen::Vector3f lastPose;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_matching_node");
    ScanMatchingNode node;
    ros::spin();
    return 0;
}
