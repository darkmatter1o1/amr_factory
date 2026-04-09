from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ===============================
    # OLD_WS – Robot + LiDAR
    # ===============================
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vbot1_v2'),
                'launch',
                'launch_robot.launch.py'
            )
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_s2e_launch.py'
            )
        )
    )

    # ===============================
    # BNO055 (run command)
    # ===============================
    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        arguments=[
            '--ros-args',
            '--params-file',
            os.path.join(
                get_package_share_directory('bno055'),
                'params',
                'bno055_params_i2c.yaml'
            )
        ]
    )

    # ===============================
    # Robot Localization EKF
    # ===============================
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_localization'),
                'launch',
                'ekf.launch.py'
            )
        )
    )

    # ===============================
    # Localization with map (OLD_WS)
    # ===============================
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vbot1_v2'),
                'launch',
                'localization_launch.py'
            )
        ),
        launch_arguments={
            'map': './ps1.yaml',
            'use_sim_time': 'false'
        }.items()
    )

    # ===============================
    # Navigation (OLD_WS)
    # ===============================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('vbot1_v2'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # ===============================
    # Custom IMU Publisher
    # ===============================
    imu_pub = Node(
        package='my_imu_publisher',
        executable='thick',
        name='imu_publisher',
        output='screen'
    )

    # ===============================
    # Orbbec Gemini 2
    # ===============================
    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch',
                'gemini2.launch.py'
            )
        ),
        launch_arguments={
            'sync_mode': 'hardware_sync',
            'ir_fps': '30',
            'depth_fps': '30',
            'color_fps': '30',
            'color_width': '640',
            'color_height': '360',
            'ir_width': '640',
            'ir_height': '400',
            'depth_width': '320',
            'depth_height': '200'
        }.items()
    )

    # ===============================
    # ArUco Pose Estimation
    # ===============================
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('aruco_pose_estimation'),
                'launch',
                'aruco_pose_estimation.launch.py'
            )
        )
    )

    # ===============================
    # Timed Sequence (delays)
    # ===============================
    return LaunchDescription([

        robot_launch,
        lidar_launch,

        TimerAction(period=5.0, actions=[bno055_node]),

        TimerAction(period=8.0, actions=[ekf_launch]),

        TimerAction(period=12.0, actions=[localization_launch]),

        # ⏱️ 10 seconds for initial pose in RViz
        TimerAction(period=22.0, actions=[navigation_launch]),

        TimerAction(period=25.0, actions=[imu_pub]),

        TimerAction(period=30.0, actions=[orbbec_launch]),

        TimerAction(period=35.0, actions=[aruco_launch]),
    ])
