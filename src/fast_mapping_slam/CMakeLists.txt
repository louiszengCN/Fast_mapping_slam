cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_CXX_STANDARD 14)

project(fm_slam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Debug)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
nav_msgs
roscpp
sensor_msgs
tf
tf_conversions
laser_geometry
)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf_conversions
  laser_geometry
  gmapping
  #hector_slam  # 确认hector_slam在这里列出
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)





# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
find_package(PkgConfig)
# find_package(PCL REQUIRED QUIET)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.10/pcl)
find_package(PCL 1.10 REQUIRED)



#find_package(hector_slam)
pkg_check_modules(csm REQUIRED csm)
## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS thread signals)
# include_directories()

# Find Eigen3 (from http://wiki.ros.org/jade/Migration)
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES lesson4
 CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
#  DEPENDS system_lib
)

###########
## Build ##
###########
#include_directories(${PCL_INCLUDE_DIRS})
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)
link_directories(${csm_LIBRARY_DIRS})





add_executable(fm_slam_node
  src/plicp_map.cc

)

add_dependencies(fm_slam_node
${catkin_EXPORTED_TARGETS}
)

target_link_libraries(fm_slam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${csm_LIBRARIES}
)

# add_executable(backend_node
#   src/backedn.cc

# )

# add_dependencies(backend_node
# ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(backend_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${csm_LIBRARIES}
# )
# add_executable(compensate_node
#   src/compensate.cc

# )

# add_dependencies(compensate_node
# ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(compensate_node
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${csm_LIBRARIES}
#   ${PCL_LIBRARY_DIRS}
#   ${PCL_LIBRARIES}
# )
# .cc要一起编译
add_executable(compensate_fm_slam_node
  src/compensate.cc
  src/plicp_map_compensate.cc
)

add_dependencies(compensate_fm_slam_node
${catkin_EXPORTED_TARGETS}
${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(compensate_fm_slam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${csm_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
  ${PCL_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR/include}
)

