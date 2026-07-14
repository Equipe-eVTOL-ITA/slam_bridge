# Runs ONLY the OAK-D Pro capture on the Jetson.
#
# The camera config is selected by the slam_backend argument:
#   slam_backend:=vslam   (default) -> camera_params_vslam.yaml
#       rectified stereo pair + IMU for Isaac ROS Visual SLAM (cuVSLAM).
#       cuVSLAM itself runs inside the Isaac ROS container - see
#       cuvslam.launch.py.
#   slam_backend:=rtabmap           -> camera_params.yaml
#       RGB + aligned depth for the RTAB-Map pipeline (rtabmap_pc.launch.py
#       on the PC), sharing the same DDS network.
#
# Requirements when splitting nodes across machines/containers:
#   - same ROS_DOMAIN_ID exported
#   - same RMW implementation (don't mix FastRTPS and CycloneDDS)
#   - clocks synced (chrony/NTP) - stamps originate on the Jetson but are
#     consumed against the PC clock (TF, slam_bridge PX4 timesync)
#   - wired gigabit link (raw 720p rgb+depth is too heavy for WiFi)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('slam_bridge')

    slam_backend = LaunchConfiguration('slam_backend')

    params_file = PathJoinSubstitution([
        pkg_dir, 'config',
        PythonExpression([
            "'camera_params_vslam.yaml' if '", slam_backend,
            "' == 'vslam' else 'camera_params.yaml'"
        ])
    ])

    # Camera driver. camera.launch.py also starts robot_state_publisher with
    # the OAK URDF, so the whole oak TF tree (base 'oak', optical frames,
    # oak_imu_frame) is published from the Jetson.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'),
                         'launch', 'camera.launch.py')
        ),
        # rectify_rgb spawns a host-side image_proc rectify node for the RGB
        # stream; pointless for the vslam (Depth pipeline, no RGB) config.
        launch_arguments={
            'params_file': params_file,
            'rectify_rgb': PythonExpression(
                ["'false' if '", slam_backend, "' == 'vslam' else 'true'"]),
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_backend', default_value='vslam',
            choices=['vslam', 'rtabmap'],
            description='Which SLAM backend the camera config should feed'),
        camera_launch,
    ])
