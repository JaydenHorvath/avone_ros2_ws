#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Optional: GPU env vars for Nvidia laptops
    env_offload = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD', value='1'
    )
    env_vendor = SetEnvironmentVariable(
        name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'
    )

    # -- Only Gazebo simulation world and robot spawn --

    # Launch Gazebo with a specific world file
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v1 /home/jay/ros2_ws/src/avone/worlds/smalltrack.world'
        }.items()
    )

    # Get robot URDF from xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([
            FindPackageShare('avone'),
            'description', 'robot.urdf.xacro'
        ])
    ])

    # Spawn robot entity into Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '--Y', '0'
        ],
        parameters=[{'robot_description': robot_description_content}]
    )

    # Gazebo <-> ROS2 sensor and data bridges
    # ros_gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     name='ros_gz_bridge',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[
    #         '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    #         '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
    #         '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist',
    #         '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    #         'rgbdcamera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
    #         'rgbdcamera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    #         'rgbdcamera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
    #         'rgbdcamera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
    #         '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
    #         '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    #         '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
    #         'camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
    #         'camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
    #         '/navsat1@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
    #         '/navsat2@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
    #     ],
    #     remappings=[
    #         ('rgbdcamera/image', '/camera/rgbd/image_raw'),
    #         ('rgbdcamera/camera_info', '/camera/rgbd/camera_info'),
    #         ('rgbdcamera/points', '/camera/rgbd/points'),
    #         ('rgbdcamera/depth_image', '/camera/rgbd/depth_image'),
    #         ('/lidar', '/scan'),
    #         ('/lidar/points', '/lidar/points'),
    #     ]
    # )

    

    # Build description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        env_offload,
        env_vendor,
        gz_sim,
        gz_spawn_entity,
        # ros_gz_bridge,
        
    ])
