cmake_minimum_required(VERSION 2.8.3)
project(wit_ros_imu)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIB_DIRS}
)

add_executable(wit_cpp src/wit.cpp)

target_link_libraries(
  wit_cpp
  ${catkin_LIBRARIES}
)

install(DIRECTORY demo driver robot_para rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

