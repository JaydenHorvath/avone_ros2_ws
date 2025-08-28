# multi_bringup.launch.py
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

       

        # --- AVONE bringup ---
        ExecuteProcess(
            cmd=['ros2', 'launch', 'avone', 'bringup.launch.py'],
            output='screen'
        ),

        # --- LiDAR Cone Mapper ---
        Node(
            package='cone_mapper',
            executable='DEMOlidarconemapper',
            name='lidar_cone_mapper',
            output='screen'
        ),

        # --- Open Day Demo ---
        Node(
            package='open_day',
            executable='open_day_demo',
            name='open_day_demo',
            output='screen'
        ),

        # --- YOLO ---
        

        # --- Static TFs ---
      
    ])
