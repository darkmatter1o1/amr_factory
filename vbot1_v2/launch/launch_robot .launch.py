import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'vbot1_v2'

    # --- Debug: Notify start of launch file
    print("[DEBUG] Starting launch_robot.launch.py")

    # Create robot_description directly
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    print(f"[DEBUG] Using xacro file: {xacro_file}")

    robot_description_content = Command([
        'xacro ', xacro_file
    ])
    robot_description = {'robot_description': robot_description_content}

    # --- Debug: Notify robot description being built
    log_robot_desc = LogInfo(msg="[DEBUG] Generated robot_description from xacro.")

    # robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )
    log_rsp_start = LogInfo(msg="[DEBUG] Launched robot_state_publisher.")

    # ros2_control_node
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )
    print(f"[DEBUG] Using controller parameters file: {controller_params_file}")

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        output='screen'
    )
    log_ctrl_node_start = LogInfo(msg="[DEBUG] Launched ros2_control_node.")

    # Controller spawners
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output="screen"
    )
    log_diff_spawner = LogInfo(msg="[DEBUG] Spawning diff_drive_controller.")

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output="screen"
    )
    log_joint_spawner = LogInfo(msg="[DEBUG] Spawning joint_state_broadcaster.")

    # Delays (to ensure proper startup order)
    delayed_ros2_control_node = TimerAction(
        period=2.0,
        actions=[log_ctrl_node_start, ros2_control_node]
    )
    delayed_diff_drive_spawner = TimerAction(
        period=4.0,
        actions=[log_diff_spawner, diff_drive_spawner]
    )
    delayed_joint_broad_spawner = TimerAction(
        period=5.0,
        actions=[log_joint_spawner, joint_broad_spawner]
    )

    # --- Debug: Notify end of launch file setup
    print("[DEBUG] Launch file setup complete. Ready to launch nodes.")

    return LaunchDescription([
        log_robot_desc,
        robot_state_publisher_node,
        log_rsp_start,
        delayed_ros2_control_node,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
