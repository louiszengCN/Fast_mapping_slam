快速建图slam
简单仿真环境搭建
src下
git clone https://github.com/6-robot/wpr_simulation.git
进入src/scripts/
执行脚本 ./install 选择自己的ROS版本
然后出去catkin_make
roslaunch wpr_simulation wpb_simple.launch
roslaunch wpr_simulation keyboard(自动补全)

rosrun fm_slam fm_slam_node

打开rviz查看建图效果