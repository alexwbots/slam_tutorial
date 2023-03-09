cmake_minimum_required(VERSION 3.0.2)
project(basic_slam_v2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES basic_slam_v2
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(main_v2 src/slam.cpp src/main.cpp)
target_link_libraries(main_v2 ${catkin_LIBRARIES})
#add_dependencies(main ${catkin_EXPORTED_TARGETS})