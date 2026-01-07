# AV.ONE Robot State Publisher Test
# File: avone_rsp.launch.py
# Launch:
#   ros2 launch avone avone_rsp.launch.py use_sim_time:=false

# Purpose:
#   - Generate robot_description from robot.urdf.xacro
#   - Start robot_state_publisher (TF tree from URDF)
#   - Start joint_state_publisher_gui for manually moving joints in RViz
#   - Optionally start RViz with a known config

# Notes:
#   - This is a pure "URDF and TF looks right" test. No Gazebo, no ros2_control.
#   - If you want sim time, set use_sim_time:=true and make sure something is publishing /clock.
#     Otherwise leave it false for desktop testing.


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import xacro


def generate_launch_description():

    # Declare sim time flag
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get xacro file and process robot description
    pkg_path = os.path.join(get_package_share_directory("avone"))
    xacro_file = os.path.join(pkg_path, "description", "robot.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    rviz_config_path = os.path.join(pkg_path, "config", "view_avone.rviz")
    params = {
        "robot_description": robot_description_config.toxml(),
        "use_sim_time": use_sim_time,
    }

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_config.toxml()},
        ],
    )

    node_joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )
    # RViz Node (without config file)

    # Optional static map->odom transform
    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_map",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use sim time if true"
            ),
            node_robot_state_publisher,
            node_joint_state_publisher,
            # static_map_tf,
            rviz_node,
        ]
    )
