from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    articubot_dir = get_package_share_directory('vbot1_v2')
    

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Delay 2 seconds before launching RPLiDAR


        TimerAction(
            period=25.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='amcl_pose',
                    name='imu_publisher_node',
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=30.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='thick',
                    name='imu_publisher_node',
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=33.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='imu_publisher',
                    name='imu_publisher',
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=36.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='logger',
                    name='logger',
                    output='screen',
                )
            ]
        ),

        



        TimerAction(
            period=40.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='battery',
                    name='battery_node',
                    output='screen',
                )
            ]
        ),

    ])
