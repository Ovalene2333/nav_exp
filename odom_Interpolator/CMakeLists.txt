cmake_minimum_required(VERSION 3.0.2)
project(odom_interpolator)  # 替换为你的包名

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf tf2_ros tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_interpolator src/odom_interpolator_node.cc)

target_link_libraries(odom_interpolator
  ${catkin_LIBRARIES}
)
target_link_libraries(odom_interpolator ${EIGEN3_LIBRARIES})