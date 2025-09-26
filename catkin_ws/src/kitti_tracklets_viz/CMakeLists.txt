cmake_minimum_required(VERSION 3.0.2)
project(kitti_tracklets_viz)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs visualization_msgs geometry_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
install(PROGRAMS scripts/tracklets_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
