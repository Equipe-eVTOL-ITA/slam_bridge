from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Get package directory and local configuration file path
    pkg_dir = get_package_share_directory('slam_bridge')
    camera_params_file = os.path.join(pkg_dir, 'config', 'camera_params.yaml')
    rtabmap_odometry_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_odometry_params.yaml')
    rtabmap_slam_params_file = os.path.join(pkg_dir, 'config', 'rtabmap_slam_params.yaml')
    
    # 1. Driver da câmera (CORRIGIDO: Passando o params_file real do pacote local)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
        ),
        # Passa o caminho absoluto do arquivo camera_params.yaml para desativar a rede MobileNet
        launch_arguments={'params_file': camera_params_file}.items()
    )

    #node para filtrar os dados do IMU antes de passar para o RTAB-map fazer o slam
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,          # OAK-D Pro não tem magnetômetro
            'publish_tf': False,       # RTAB-Map gerencia o TF
            'world_frame': 'enu',
            # Frame real publicado pelo driver (URDF): oak_imu_frame (underscore)
            'fixed_frame': 'oak_imu_frame',
            'base_link_frame': 'oak_imu_frame',
            'gain': 0.01,              # Ganho baixo = confia mais no gyro
            'zeta': 0.0,
            'frequency': 200.0,        # OAK-D Pro IMU roda a 200Hz
            'publish_debug_topics': False,
        }],
        remappings=[
            ('imu/data_raw', '/oak/imu/data'),   # tópico bruto da OAK-D
            ('imu/data',     '/imu/data'),        # saída filtrada
        ]
    )

    #Nó de ODOMETRIA VISUAL (Janela de Sincronização Expandida)
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

    # Nó de MAPEAMENTO (RTAB-Map)
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
        # arguments=['-d'] 
    )

    # NOTA: o TF do IMU (oak_imu_frame) ja e publicado pelo driver via URDF
    # com a rotacao correta de calibracao. Um static_transform_publisher
    # manual com rotacao identidade a partir do frame OPTICO (z-forward)
    # estava errado e conflitava com o TF do driver. Verifique com:
    #   ros2 run tf2_ros tf2_echo oak oak_imu_frame

    # SLAM Bridge node - converts RTAB-Map odometry to PX4 format (delayed start)
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
        camera_launch,
        imu_filter_node,
        visual_odometry_node,
        rtabmap_node,
        slam_bridge_node
    ])