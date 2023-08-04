#include "../include/sub_map.h"
// 发布子图
submap_creater::submap_creater(){
    // map params
    resolution_sub = 0.05;
    map_width_sub = 900;
    map_height_sub = 900;
    //map_origin_x _y 就是map在rviz的右上角坐标，rviz中默认一个格子是1，记得乘以0.05变换成栅格坐标
    map_origin_x_sub = -22.5;//-450*0.05 -450是map的中心点
    map_origin_y_sub = -22.5;
    occupied_threshold_sub = 0.7;
    free_threshold_sub = 0.3;
    grid_map_sub = std::vector<std::vector<int>>(map_height_sub, std::vector<int>(map_width_sub, 50));
    // map params
    map_msg_sub.header.frame_id = "map";
    map_msg_sub.info.width = map_width_sub;
    map_msg_sub.info.height = map_height_sub;
    map_msg_sub.info.resolution = resolution_sub;
    map_msg_sub.info.origin.position.x = map_origin_x_sub;
    map_msg_sub.info.origin.position.y = map_origin_y_sub;
    map_msg_sub.info.origin.orientation.w = 1.0;

}

std::vector<Point_sub> submap_creater::Bresenham_sub(int x0, int y0, int x1, int y1){

 std::vector<Point_sub> points;

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


nav_msgs::OccupancyGrid submap_creater::CreateSubMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg, double r_x, double r_y, double r_theta)
{
temp_angle_increment_sub = scan_msg->angle_increment;
temp_angle_min_sub = scan_msg->angle_min;
std::vector<float> ranges = scan_msg->ranges;
    
for (size_t i = 0; i < ranges.size(); ++i) {
    // 跳过无效点
    if(ranges[i]<scan_msg->range_min || ranges[i]>scan_msg->range_max)
    {
        continue;
    }
    angle_robot_sub = temp_angle_min_sub + i * temp_angle_increment_sub;
    // 雷达hit点的坐标 laser坐标系下
    x_robot_sub = ranges[i] * cos(angle_robot_sub + r_theta) + r_x;
    y_robot_sub = ranges[i] * sin(angle_robot_sub + r_theta) + r_y;
    // // 雷达hit点坐标转换到栅格坐标系
    // grid_x = static_cast<int>((x_robot - map_origin_x) / resolution) + 450;
    // grid_y = static_cast<int>((y_robot - map_origin_y) / resolution) + 450;
    // // 找到机器人在地图中的位置 450是地图的中点坐标(900*900)
    // robot_location_x = static_cast<int>((robot_pose_x - map_origin_x) / resolution) + 450;
    // robot_location_y = static_cast<int>((robot_pose_y - map_origin_y) / resolution) + 450;
        
    // 这里不需要加偏移量，否则地图就不会在base_link脚下更新，会在偏移处更新，无法做到在机器人脚下建图
    // 雷达hit点坐标转换到栅格坐标系
    grid_x_sub = static_cast<int>((x_robot_sub - map_origin_x_sub) / resolution_sub) ;
    grid_y_sub = static_cast<int>((y_robot_sub - map_origin_y_sub) / resolution_sub) ;
    // 找到机器人在地图中的位置 450是地图的中点坐标(900*900)
    robot_location_x_sub = static_cast<int>((r_x - map_origin_x_sub) / resolution_sub) ;
    robot_location_y_sub = static_cast<int>((r_y - map_origin_y_sub) / resolution_sub) ;
    //grid_x grid_y是击中的坐标
    if (grid_x_sub >= 0 && grid_x_sub < map_width_sub && grid_y_sub >= 0 && grid_y_sub < map_height_sub) {
        //把击中的赋值100
        if(grid_map_sub[grid_y_sub][grid_x_sub]<100)
        {
        grid_map_sub[grid_y_sub][grid_x_sub] += 5;
        }
        else
        {
        grid_map_sub[grid_y_sub][grid_x_sub] = 100;
        }
        // 没有击中的赋值0
        std::vector<Point_sub> v1 = Bresenham_sub(robot_location_x_sub, robot_location_y_sub, grid_x_sub, grid_y_sub);
        // V1是空闲的点
        for(int i = 0; i < v1.size(); i++)
        {
        if(grid_map_sub[v1[i].y][v1[i].x]>0)
        {
        grid_map_sub[v1[i].y][v1[i].x] += -1;
        }
        else
        {
        grid_map_sub[v1[i].y][v1[i].x] = 0;
        }
        
        }
            
        }
    }

std::vector<int8_t> flat_map;
for (const auto& row : grid_map_sub) {
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
map_msg_sub.data = flat_map;
map_msg_sub.header.stamp = ros::Time::now();
return map_msg_sub;

}
