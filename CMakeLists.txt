cmake_minimum_required(VERSION 3.16)
project(slam_bridge)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(slam_bridge
src/slam_bridge.cpp
)

ament_target_dependencies(slam_bridge
  rclcpp geometry_msgs nav_msgs px4_msgs Eigen3)

add_executable(rect_camera_info_fixer
src/rect_camera_info_fixer.cpp
)

ament_target_dependencies(rect_camera_info_fixer
  rclcpp sensor_msgs)

install(TARGETS slam_bridge rect_camera_info_fixer DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
