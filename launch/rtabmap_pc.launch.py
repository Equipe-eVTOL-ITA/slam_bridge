# Runs the processing half of the pipeline on the PC: IMU filter,
# RTAB-Map visual odometry, RTAB-Map SLAM and the PX4 slam_bridge.
# The camera must already be running on the Jetson via camera_jetson.launch.py.
#
# slam_bridge lives here because in the current debug setup the uXRCE-DDS
# agent (PX4 link) runs on this PC. When everything moves onboard, use the
# all-in-one rtabmap.launch.py instead.
#
# See camera_jetson.launch.py for the cross-machine requirements
# (ROS_DOMAIN_ID, same RMW, chrony clock sync, wired link).

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('slam_bridge')
    rtabmap_odometry_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_odometry_params.yaml')
    rtabmap_slam_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_slam_params.yaml')

    # Filters the raw OAK IMU (published from the Jetson) into an
    # orientation estimate for RTAB-Map.
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,          # OAK-D Pro has no magnetometer
            'publish_tf': False,       # RTAB-Map manages TF
            'world_frame': 'enu',
            # Real frame published by the driver URDF: oak_imu_frame
            'fixed_frame': 'oak_imu_frame',
            'base_link_frame': 'oak_imu_frame',
            'gain': 0.01,              # Low gain = trust the gyro more
            'zeta': 0.0,
            'frequency': 200.0,        # OAK-D Pro IMU runs at 200Hz
            'publish_debug_topics': False,
        }],
        remappings=[
            ('imu/data_raw', '/oak/imu/data'),   # raw topic from the OAK-D
            ('imu/data',     '/imu/data'),        # filtered output
        ]
    )

    # Visual odometry
    visual_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[rtabmap_odometry_params_file],
        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('odom', '/odom_local'),
            ('imu', '/imu/data'),
        ]
    )

    # Mapping (RTAB-Map)
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_slam_params_file],
        remappings=[
            ('rgb/image', '/oak/rgb/image_raw'),
            ('rgb/camera_info', '/oak/rgb/camera_info'),
            ('depth/image', '/oak/stereo/image_raw'),
            ('odom', '/odom_local'),
            ('imu', '/imu/data'),
        ],
        arguments=['-d']
    )

    # SLAM Bridge - converts RTAB-Map odometry to PX4 format (delayed start)
    slam_bridge_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='slam_bridge',
            executable='slam_bridge',
            name='slam_bridge',
            output='screen',
            parameters=[{
                'slam_odometry_topic': '/odom_local',
                'px4_odometry_output_topic': '/fmu/in/vehicle_visual_odometry',
                'timesync_topic': '/fmu/out/timesync_status',
                'variance_floor': 0.1,
            }]
        )]
    )

    return LaunchDescription([
        imu_filter_node,
        visual_odometry_node,
        rtabmap_node,
        slam_bridge_node,
    ])
