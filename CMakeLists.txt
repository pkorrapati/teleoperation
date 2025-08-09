cmake_minimum_required(VERSION 3.0.2)
project(steering)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  steering_data  
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES steering
#  CATKIN_DEPENDS rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(steerforce src/steerforce.cpp)
target_link_libraries(steerforce ${catkin_LIBRARIES})