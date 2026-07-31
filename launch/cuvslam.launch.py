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
# IMU fusion is OFF by default: the BNO086 noise densities below are
# datasheet ballparks, NOT Kalibr-calibrated values. Pass
# enable_imu_fusion:=true to experiment anyway.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            # left_rect/right_rect are rectified on-device by StereoDepth
            'rectified_images': True,

            # Pure visual odometry for the PX4 EKF2 feed - no map building,
            # no loop closure (a loop-closure pose jump mid-flight would be
            # worse for EKF2 than smooth VO drift).
            'enable_localization_n_mapping': True,

            # IMU (BNO086) - UNCALIBRATED datasheet ballparks, see header.
            'enable_imu_fusion': enable_imu_fusion,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.00226,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,

            # 30 fps pair -> nominal 33.3 ms period
            'image_jitter_threshold_ms': 35.0,

            # Frames from the OAK URDF published by the host-side driver
            'base_frame': 'oak',
            'imu_frame': 'oak_imu_frame',
            'camera_optical_frames': [
                'oak_left_camera_optical_frame',
                'oak_right_camera_optical_frame',
            ],

            # Headless on the Jetson; flip on for RViz debugging sessions
            'enable_slam_visualization': False,
            'enable_landmarks_view': False,
            'enable_observations_view': False,
        }],
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
