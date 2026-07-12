# Runs ONLY the OAK-D Pro capture on the Jetson. Everything else
# (imu filter, rgbd odometry, rtabmap, slam_bridge) runs on the PC via
# rtabmap_pc.launch.py, sharing the same DDS network.
#
# Requirements for the split to work (both machines):
#   - same ROS_DOMAIN_ID exported
#   - same RMW implementation (don't mix FastRTPS and CycloneDDS)
#   - clocks synced (chrony/NTP) - stamps originate on the Jetson but are
#     consumed against the PC clock (TF, slam_bridge PX4 timesync)
#   - wired gigabit link (raw 720p rgb+depth is too heavy for WiFi)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('slam_bridge')
    camera_params_file = os.path.join(pkg_dir, 'config', 'camera_params.yaml')

    # Camera driver. camera.launch.py also starts robot_state_publisher with
    # the OAK URDF, so the whole oak TF tree (base 'oak', optical frames,
    # oak_imu_frame) is published from the Jetson.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'),
                         'launch', 'camera.launch.py')
        ),
        launch_arguments={'params_file': camera_params_file}.items()
    )

    return LaunchDescription([
        camera_launch,
    ])
