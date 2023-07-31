# Fast_mapping_slam
A light slam algorithm which runs five times faster than hector slam

### To quickly start, you should follow:
1. #### create simulation environment
```shell
cd /src
git clone https://github.com/6-robot/wpr_simulation.git
cd /wpr_simulation/scripts
./install_for_(your ROS version noetic/melodic/kinetic).sh
```
back to your root
```
catkin_make
```
2. ### start simulation environment
```shell
# start simulation environment
roslaunch wpr_simulation wpb_stage_robocup.launch
# start keyboard control node
rosrun wpr_simulation keyboard_vel_ctrl
# start fast_mapping_slam
rosrun fm_slam fm_slam_node
```
3. ### view map in rviz
```shell
rviz
```

TF TREE:  
```C
MAP  
 |---odom_plicp   
         |---base_link         
                |---laser  
```

Advertised topic:  

```C
/odom_plicp //nav_msgs::Odometry
/path_topic //nav_msgs::Path
/occupancy_grid //nav_msgs::OccupancyGrid
/tf 
```
