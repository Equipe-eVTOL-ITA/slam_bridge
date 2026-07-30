# Runs the OAK-D Pro capture on the Jetson for the cuVSLAM pipeline:
# rectified stereo pair + IMU, IR dot projector off (camera_params_vslam.yaml).
# cuVSLAM itself runs inside the Isaac ROS container - see cuvslam.launch.py.
#
# (RTAB-Map used to share this launch file via a slam_backend arg, camera on
# the Jetson and processing on a separate PC over the network - that split
# was a failed experiment, too much bandwidth for the raw RGB+depth stream.
# RTAB-Map now runs all-in-one via rtabmap.launch.py instead.)
#
# Requirements when splitting nodes across machines/containers:
#   - same ROS_DOMAIN_ID exported
#   - same RMW implementation (don't mix FastRTPS and CycloneDDS)
#   - clocks synced (chrony/NTP) - stamps originate on the Jetson but are
#     consumed against the container clock (TF, slam_bridge PX4 timesync)

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('slam_bridge')
    params_file = os.path.join(pkg_dir, 'config', 'camera_params_vslam.yaml')

    # Camera driver. camera.launch.py also starts robot_state_publisher with
    # the OAK URDF, so the whole oak TF tree (base 'oak', optical frames,
    # oak_imu_frame) is published from the Jetson.
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'),
                         'launch', 'camera.launch.py')
        ),
        # rectify_rgb spawns a host-side image_proc rectify node for the RGB
        # stream; pointless here (Depth pipeline, no RGB).
        launch_arguments={
            'params_file': params_file,
            'rectify_rgb': 'false',
        }.items()
    )

    return LaunchDescription([
        camera_launch,
    ])
