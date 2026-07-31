# Host-side half of the cuVSLAM pipeline - the Isaac ROS counterpart of
# rtabmap.launch.py. Starts on the Jetson host:
#
#   1. OAK-D Pro driver with the vslam camera config (rectified stereo
#      pair + IMU, IR dot projector off)
#   2. slam_bridge pointed at cuVSLAM's odometry output
#
# The cuVSLAM node itself CANNOT run here - isaac_ros_visual_slam only
# exists inside the Isaac ROS container. Start it there separately:
#
#   cd src/isaac_ros_common && ./scripts/run_dev.sh -d /home/jetson-evtol/evtol/dev
#   (inside) ros2 launch /workspaces/isaac_ros-dev/src/slam_bridge/launch/cuvslam.launch.py
#
# Everything shares DDS: the container runs with --network host.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('slam_bridge')

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'camera_jetson.launch.py')
        )
    )

    camera_info_fixer = Node(
        package='slam_bridge',
        executable='rect_camera_info_fixer',
        name='rect_camera_info_fixer',
        output='screen',
        parameters=[{
            'left_input_topic': '/oak/left/camera_info',
            'right_input_topic': '/oak/right/camera_info',
            'left_output_topic': '/vslam/left/camera_info',
            'right_output_topic': '/vslam/right/camera_info',
        }]
    )

    # SLAM Bridge node - converts cuVSLAM odometry to PX4 format (delayed
    # start so the camera/TF tree is up first)
    slam_bridge_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='slam_bridge',
            executable='slam_bridge',
            name='slam_bridge',
            output='screen',
            parameters=[{
                'slam_odometry_topic': '/visual_slam/tracking/odometry',
                'px4_odometry_output_topic': '/fmu/in/vehicle_visual_odometry',
                'timesync_topic': '/fmu/out/timesync_status',
                'variance_floor': 0.1,
            }]
        )]
    )

    return LaunchDescription([
        camera_launch,
        camera_info_fixer,
        slam_bridge_node,
    ])
