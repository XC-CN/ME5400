cmake_minimum_required(VERSION 3.0.2)
project(ME5400)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_filters
  message_generation
  tf2_ros
)

add_message_files(
  FILES
    Detection3D.msg
    Detection3DArray.msg
    TrackedObject.msg
    TrackedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    visualization_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    message_runtime
    message_filters
    tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS
  scripts/tracklets_publisher.py
  scripts/mctrack_marker_publisher.py
  scripts/fastlio_pose_recorder.py
  scripts/odom_to_tum_recorder.py
  scripts/fastlio_pose_bridge.py
  scripts/mctrack_online_node.py
  scripts/joint_backend_ego_node.py
  scripts/offline_bag_feeder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
