#!/usr/bin/env python3
"""
AV.ONE NUC Boot Launch
----------------------
Launches the DBC CAN bridge node
Launches the LED matrix controller node
Opens Terminator fullscreen running the AV.ONE Dashboard
"""

from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # === Paths ===
    dbc_path = '/home/avone/NUTEAMSGIT/NUCAN/DBC Files/AV1.dbc'

    # === DBC CAN bridge ===
    dbc_can_bridge = Node(
        package='avone_can',
        executable='can_2_ros',
        name='dbc_can_bridge',
        output='screen',
        parameters=[{
            'dbc_file': dbc_path,
            'dbc_path': dbc_path
        }],
    )

    # === LED Matrix ===
    drive_state_led = Node(
        package='avone_utils',  # adjust if different
        executable='drive_state_led',
        name='drive_state_led',
        output='screen',
    )

    nuc_hearbeats = Node(
        package='avone_utils',  # adjust if different
        executable='ros_cmd_heartbeat',
        name='ros_cmd_heartbeat',
        output='screen',
    )
        # === LED Matrix ===
    sensor_timout = Node(
        package='avone_utils',  # adjust if different
        executable='sensor_timeout',
        name='sensor_timeout',
        output='screen',
    )


    # === Launch sequence ===
    return LaunchDescription([
        LogInfo(msg='[AV.ONE] Launching CAN bridge...'),
        dbc_can_bridge,
        LogInfo(msg='[AV.ONE] Launching LED matrix controller...'),
        drive_state_led,
        LogInfo(msg='[AV.ONE] Launching NUC Heartbeats...'),
        nuc_hearbeats,
        LogInfo(msg='[AV.ONE] Launching Sensor Timouts...'),
        sensor_timout,

        
      
    ])