cmake_minimum_required(VERSION 3.0.2)
project(pid_local_planner)  # 替换为你的包名

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf tf2_ros tf2_geometry_msgs visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pid_controller_node src/pid_controller_node_world.cc)
add_executable(path_generator_node src/path_generator_node.cc)

# add_executable(pid_controller_node src/pid_controller_node.cc)

target_link_libraries(path_generator_node
  ${catkin_LIBRARIES}
)

target_link_libraries(pid_controller_node
  ${catkin_LIBRARIES}
)