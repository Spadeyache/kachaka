cmake_minimum_required(VERSION 3.8)
project(tf2_addon)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(rgbd_tf_publisher static-src/rgbd_tf_publisher.cpp)
ament_target_dependencies(rgbd_tf_publisher
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

install(TARGETS
  rgbd_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
