cmake_minimum_required(VERSION 3.16)
project(slam_bridge)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(slam_bridge
src/slam_bridge.cpp
)

ament_target_dependencies(slam_bridge
  rclcpp geometry_msgs nav_msgs px4_msgs Eigen3)

install(TARGETS slam_bridge DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install config files
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
