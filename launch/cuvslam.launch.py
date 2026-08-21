# Isaac ROS Visual SLAM (cuVSLAM) fed by the OAK-D Pro rectified stereo
# pair. Runs INSIDE the Isaac ROS container (the only place the
# isaac_ros_visual_slam package exists), while the camera driver
# (camera_jetson.launch.py) and slam_bridge run on the Jetson host - both
# sides share DDS because the container uses
# --network host.
#
# The container mounts this workspace at /workspaces/isaac_ros-dev, and this
# package is not built inside the container, so launch it by file path:
#
#   ros2 launch /workspaces/isaac_ros-dev/src/slam_bridge/launch/cuvslam.launch.py
#
# isaac_ros_visual_slam is baked into the container image by
# src/slam_bridge/docker/Dockerfile.evtol - no manual apt install. If the
# node fails to load, run src/slam_bridge/scripts/setup_isaac_container.sh
# --build (the image layer is probably not wired up on this machine yet).
#
# Parameters live in ../config/cuvslam_params.yaml. Resolved relative to this
# file's own path, NOT via get_package_share_directory() - this package is not
# built inside the container.
#
# STARTUP ORDER MATTERS. The OAK TF tree (base 'oak', optical frames) is
# published by robot_state_publisher on the HOST, as a single latched
# /tf_static message. If this node starts before the host side, cuVSLAM's
# extrinsics lookups fire before DDS discovery has delivered that message and
# you get a burst of:
#
#     Warning: Invalid frame ID "oak" passed to canTransform argument
#     target_frame - frame does not exist
#
# Start vslam.launch.py on the host FIRST, confirm the frame is really there
#     ros2 topic echo /tf_static --once
#     ros2 run tf2_ros tf2_echo oak oak_left_camera_optical_frame
# and only then launch this. See scripts/kill_slam.sh before relaunching.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():

    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')

    # This package is not built inside the Isaac ROS container, so the config
    # is found relative to this launch file rather than via ament.
    params_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'cuvslam_params.yaml')

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[
            params_file,
            # launch-arg override; everything else comes from the YAML
            {'enable_imu_fusion': enable_imu_fusion},
        ],
        remappings=[
            # camera_info comes from rect_camera_info_fixer (host side), NOT
            # straight from the driver: /oak/{left,right}/camera_info carry
            # each sensor's raw unrectified intrinsics even on the rectified
            # topics, and cuVSLAM reads K from them. See
            # src/rect_camera_info_fixer.cpp for the measurements.
            ('visual_slam/image_0', '/oak/left/image_rect'),
            ('visual_slam/camera_info_0', '/vslam/left/camera_info'),
            ('visual_slam/image_1', '/oak/right/image_rect'),
            ('visual_slam/camera_info_1', '/vslam/right/camera_info'),
            ('visual_slam/imu', '/oak/imu/data'),
        ],
    )

    container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_imu_fusion', default_value='true',
            description='Fuse the (uncalibrated) OAK-D IMU into cuVSLAM'),
        container,
    ])
