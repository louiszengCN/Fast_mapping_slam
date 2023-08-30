#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
struct p_w_cur
{
    int index;
    float curvRange;
};



class Feature_extract
{
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher feature_scan_publisher_;

    float edge_threshold_;

public:
    Feature_extract();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

};

Feature_extract::Feature_extract()
{
    ROS_INFO_STREAM("\033[1;32m----> Feature Extraction Started.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe("/scan", 1, &Feature_extract::scanCallback, this);

    feature_scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("feature_point", 1, this);
    
    edge_threshold_ = 1.0;

}


void Feature_extract::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    
    int scan_size = scan->ranges.size();
    //存一个点和这个点的曲率 vector容器要初始化空间大小
    std::vector<p_w_cur> cur_point(scan_size);
    
    // int scan_size = scan->ranges.size();
    for (int i = 5; i < scan_size - 5; i++)
    {
        // 剔除无效点
        if(!std::isfinite(scan->ranges[i]))
        {
            continue;
        }
        // 曲率计算
        float curvRate = scan->ranges[i-5] + scan->ranges[i-4] + scan->ranges[i-3]
                                           + scan->ranges[i-2] + scan->ranges[i-1]
                                           + scan->ranges[i+5] + scan->ranges[i+4]
                                           + scan->ranges[i+3] + scan->ranges[i+2]
                                           + scan->ranges[i+1] - scan->ranges[i]*10;
        // 保存当前点曲率及索引
        cur_point[i].curvRange = curvRate * curvRate;
        cur_point[i].index = i;
    }
    // 提取后的scan
    sensor_msgs::LaserScan feature_scan;
    feature_scan.header = scan->header;
    feature_scan.angle_increment = scan->angle_increment;
    feature_scan.angle_max = scan->angle_max;
    feature_scan.angle_min = scan->angle_min;
    feature_scan.range_max = scan->range_max;
    feature_scan.range_min = scan->range_min;
    // 记得ranges也要初始化
    feature_scan.ranges.resize(scan->ranges.size());
    // 把一圈scan划分区间
    for (int j = 0; j < 6; j++)
    {
        //0~60 61~120 121~180 181~240 241~300 301~360
        int start_index = j*60 + 1;
        int end_index = j*60 + 60;

        if(start_index >= end_index)
        {
            continue;
        }
        // 遍历这个区间
        int count = 0;
        for(start_index; start_index<=end_index; start_index++)
        {
            // 限定提取数量
            if(count >= 20)
            {
                break;
            }
            // 提取角点
            if(cur_point[start_index].curvRange > edge_threshold_)
            {
                feature_scan.ranges[start_index] = scan->ranges[cur_point[start_index].index];
                count ++;
            }
        }
    }
    
    feature_scan_publisher_.publish(feature_scan);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_extract"); // 节点的名字
    Feature_extract fe;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}

















