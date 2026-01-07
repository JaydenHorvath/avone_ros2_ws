#!/usr/bin/env python3

# AV.ONE NUC Boot Launch
# File: avone_nuc_boot.launch.py
# Launch Command: ros2 launch avone avone_nuc_boot.launch.py


# Purpose:
#   - Start the core “always-on” NUC ROS processes on boot:
#       1) CAN DBC bridge (CAN -> ROS topics)
#       2) Drive state LED controller
#       3) ROS command heartbeat transmitter
#       4) Sensor timeout / watchdog node

# Notes:
#   - This launch file is started by the systemd unit:
#       /etc/systemd/system/avone_startup.service
#   - CAN interface setup is handled by a separate systemd unit:
#       /etc/systemd/system/can-setup.service


from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # Paths - hardcoded to make it easier with git, could probably be better solution
    dbc_path = "/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc"

    # CAN -> ROS DBC bridge
    dbc_can_bridge = Node(
        package="avone_can",
        executable="can_2_ros",
        name="dbc_can_bridge",
        output="screen",
        parameters=[{"dbc_file": dbc_path, "dbc_path": dbc_path}],
    )

    # Drive state LED controller
    drive_state_led = Node(
        package="avone_utils",
        executable="drive_state_led",
        name="drive_state_led",
        output="screen",
    )

    # ROS System safety heartbeats
    nuc_hearbeats = Node(
        package="avone_utils",
        executable="ros_cmd_heartbeat",
        name="ros_cmd_heartbeat",
        output="screen",
    )
    # Sensor timeout watchdog
    sensor_timout = Node(
        package="avone_utils",
        executable="sensor_timeout",
        name="sensor_timeout",
        output="screen",
    )

    # Launch Sequence
    return LaunchDescription(
        [
            LogInfo(msg="[AV.ONE] Launching CAN bridge..."),
            dbc_can_bridge,
            LogInfo(msg="[AV.ONE] Launching LED matrix controller..."),
            drive_state_led,
            LogInfo(msg="[AV.ONE] Launching NUC Heartbeats..."),
            nuc_hearbeats,
            LogInfo(msg="[AV.ONE] Launching Sensor Timouts..."),
            sensor_timout,
        ]
    )
