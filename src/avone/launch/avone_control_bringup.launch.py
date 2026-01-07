# AV.ONE Control Bringup (Simulation + ROS 2 Control)
# File: avone_control_bringup.launch.py
# Launch Command: ros2 launch avone avone_control_bringup.launch.py

# Purpose:
#   - Generate robot_description from Xacro
#   - Start ros2_control controller_manager (ros2_control_node)
#   - Start robot_state_publisher
#   - Spawn joint_state_broadcaster then ackermann_steering_controller
#   - Optionally launch RViz after JSB is active
#   - Relay controller TF odometry into /tf
#   - Provide teleop input + cmd_vel filtering

# Notes:
#   - This is a "control-centric" bringup: it focuses on robot_description + controllers.
#   - Start order matters: JSB should come up before the steering controller.
#   - tf_odometry_relay is used because the controller publishes a dedicated TF odom topic.
#   - Localisation nodes are not launched here, so


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ============================================================
    # Launch arguments
    # ============================================================
    use_sim_time = LaunchConfiguration("use_sim_time")
    can_interface = LaunchConfiguration("can_interface")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("can_interface", default_value="can0"),
    ]

    # ============================================================
    # Paths
    # ============================================================
    xacro_exe = PathJoinSubstitution([FindExecutable(name="xacro")])

    # Robot description xacro entry point
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("avone"), "description", "robot.urdf.xacro"]
    )

    # Controller config
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("avone"), "config", "ackermann_drive_controller.yaml"]
    )

    rviz_config_path = os.path.expanduser("~/ros2_ws/src/avone/config/bringup.rviz")

    # ============================================================
    # robot_description (Xacro -> URDF string)
    # ===========================================================
    robot_description_cmd = Command(
        [
            xacro_exe,
            " ",
            xacro_file,
            " ",
            "use_sim_time:=",
            use_sim_time,
            " ",
            "can_interface:=",
            can_interface,
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_cmd, value_type=str)
    }

    # ============================================================
    # Core nodes
    # ============================================================

    # controller_manager / ros2_control
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers, {"update_rate": 50}],
        output="both",
    )

    # robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    # ============================================================
    # Controller spawners
    # ============================================================

    # Joint State Broadcaster (always spawn first)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Ackermann steering controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # ============================================================
    # Utilities
    # ============================================================

    # The ackermann_steering_controller can publish TF odometry on a separate topic.
    # This relay pushes it into /tf for downstream consumers.
    tf_odometry_relay = Node(
        package="topic_tools",
        executable="relay",
        name="tf_odometry_relay",
        arguments=["/ackermann_steering_controller/tf_odometry", "/tf"],
        output="screen",
    )

    # Filter cmd_vel before it hits control stack
    cmd_filter_node = Node(
        package="avone_utils",
        executable="cmd_vel_filter",
        name="cmd_vel_filter",
        output="screen",
    )

    # ---- Start order controls ----
    delay_rviz_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_ctrl_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        *declare_args,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_jsb,
        delay_ctrl_after_jsb,
        tf_odometry_relay,
        cmd_filter_node,
    ]

    return LaunchDescription(nodes)
