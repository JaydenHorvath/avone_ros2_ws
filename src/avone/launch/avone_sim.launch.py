#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
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
    # Use /clock if running in Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Paths to your YAMLs in avone/config
    ekf_yaml = PathJoinSubstitution([FindPackageShare('avone'), 'config', 'ekf.yaml'])
    map_ekf_yaml = PathJoinSubstitution([FindPackageShare('avone'), 'config', 'map_ekf.yaml'])

    # Environment variables for GPU offload
    env_offload = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD',
        value='1'
    )
    env_vendor = SetEnvironmentVariable(
        name='__GLX_VENDOR_LIBRARY_NAME',
        value='nvidia'
    )

    # Robot description via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('avone'),
            'description', 'robot.urdf.xacro'
        ])
    ])
    robot_description = {
        'robot_description': robot_description_content,
        'use_sim_time': use_sim_time
    }

    # Controllers configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare('avone'),
        'config', 'ackermann_drive_controller.yaml'
    ])

    # RViz configuration
    rviz_config_path = os.path.expanduser(
        '~/ros2_ws/src/avone/config/view_bot.rviz'
    )

    # -------------------------------------------------------------------------
    # robot_localization Nodes
    # -------------------------------------------------------------------------
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            ekf_yaml
        ],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frequency': 30.0, 'publish_filtered_gps': False},
             {'broadcast_utm_transform': True},
            
        ],
         remappings=[
            ('gps/fix', '/navsat1'),
            ('odometry/filtered', '/odometry/local'),
            ('odometry/gps', '/odometry/gps'),
         ]
    )

    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            map_ekf_yaml
        ],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ]
    )

    static_map_tf =      Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_map',
            output='screen',
            arguments=[
                '0', '0', '0',   # x y z
                '0', '0', '0',   # roll pitch yaw
                'map', 'odom'    # parent_frame child_frame
            ]
        )

    # -------------------------------------------------------------------------
    # Gazebo bridge, state publisher, and related Nodes
    # -------------------------------------------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            'rgbdcamera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            'rgbdcamera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            'rgbdcamera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            'rgbdcamera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            'camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            'camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/navsat1@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/navsat2@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            # '/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # '/depth/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            # '/depth/image_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            ('rgbdcamera/image', '/camera/rgbd/image_raw'),
            ('rgbdcamera/camera_info', '/camera/rgbd/camera_info'),
            ('rgbdcamera/points', '/camera/rgbd/points'),
            ('rgbdcamera/depth_image', '/camera/rgbd/depth_image'),
            ('/lidar', '/scan'),
            ('/lidar/points', '/lidar/points'),
        ]
    )



    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r -v1 /home/jay/ros2_ws/src/avone/worlds/random_cylinders.sdf'
        }.items()
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'my_robot',
            '-topic', 'robot_description',
            # small track
            # '-x', '-14.8',
            # '-y', '11',
            # '-z', '0.1',
            # '--Y', '0'

            '-x', '0',
            '-y', '0',
            '-z', '0',
            '--Y', '0'

            # # small track
            # '-x', '-10.0',
            # '-y', '11',
            # '-z', '0.1',
            # '--Y', '0'
            # accel
            # '-x', '25.2118',
            # '-y', '-0.2167',
            # '-z', '0.1',
            # '--Y', '3.14'
        ]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    ackermann_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )

    # RViz (delayed until simulation is ready)
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-d', rviz_config_path]
            )
        ]
    )

    # tf_odometry_relay = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='tf_odometry_relay',
    #     arguments=[
    #         '/ackermann_steering_controller/tf_odometry',  # incoming topic
    #         '/tf'                                           # outgoing topic
    #     ],
    #     output='screen'
    # )

    # -------------------------------------------------------------------------
    # Teleop Nodes (joy + teleop_twist_joy)
    # -------------------------------------------------------------------------
    # twiststamped_node =     Node(
    #     package='avone',
    #     executable='twiststamped',
    #     name='twiststamped',
    #     output='screen'
    # )
    
    cmd_filter_node = Node(
        package='avone_utils',
        executable='cmd_vel_filter',
        name='cmd_vel_filter',
        output='screen'


    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )

    # teleop_twist_joy_node = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_twist_joy_node',
    #     output='screen',
    #     parameters=[{
    #         'stamp': True,
    #         'axis_linear.x': 1,
    #         'scale_linear.x': 2.0,
    #         'axis_angular.yaw': 3,
    #         'scale_angular.yaw': 0.5,
    #         'enable_button': 4,
    #         'repeat_rate': 50.0
    #     }],
    # )

    # -------------------------------------------------------------------------
    # Build the combined LaunchDescription
    # -------------------------------------------------------------------------
    return LaunchDescription([
        # ----------------------------------------------------
        # 1) Argument
        # ----------------------------------------------------
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),

        # ----------------------------------------------------
        # 2) GPU environment variables
        # ----------------------------------------------------
        env_offload,
        env_vendor,

        # ----------------------------------------------------
        # 3) Robot State Publisher
        # ----------------------------------------------------
        rsp_node,

        # ----------------------------------------------------
        # 4) ROS-GZ Bridge
        # ----------------------------------------------------
        ros_gz_bridge,

        # ----------------------------------------------------
        # 5) Gazebo simulation
        # ----------------------------------------------------
        gz_sim,

        # ----------------------------------------------------
        # 6) Spawn the robot entity
        # ----------------------------------------------------
        gz_spawn_entity,

        # ----------------------------------------------------
        # 7) ros2_control + controllers
        # ----------------------------------------------------
        ros2_control_node,

        # ----------------------------------------------------
        # 8) DepthImage → LaserScan
        # ----------------------------------------------------
   

        # ----------------------------------------------------
        # 9) Sequence controller spawners
        # ----------------------------------------------------
        RegisterEventHandler(
            OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_spawner]
            )
        ),

        # ----------------------------------------------------
        # 10) robot_localization Nodes
        # ---------------------------------------------------- 
         static_map_tf,
        # ekf_local_node,
        # ekf_map_node,
        # navsat_transform_node,
        
          
    #    tf_odometry_relay,
        # ----------------------------------------------------
        # 11) RViz (delayed)
        # ----------------------------------------------------
        rviz_node,

        # ----------------------------------------------------
        # 12) Teleop: joy_node & teleop_twist_joy_node
        # ----------------------------------------------------
        joy_node,
        cmd_filter_node,
        # twiststamped_node,
        # teleop_twist_joy_node,
    ])