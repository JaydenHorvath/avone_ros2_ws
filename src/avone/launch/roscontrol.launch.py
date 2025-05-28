import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

rviz_config_path = os.path.expanduser('~/ros2_ws/src/avone/config/view_bot.rviz')

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',                    # or 'ros_ign_bridge' if that’s what you’ve installed
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            # core topics
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
             '/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
             '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
           
                    
            # rgbd

              # point cloud
            'rgbdcamera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',

             # camera‐info
            'rgbdcamera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # depth image
            'rgbdcamera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',             
           'rgbdcamera/image@sensor_msgs/msg/Image[ignition.msgs.Image',


             # IMU (if you want it too)
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',

            # LIDAR: raw scan + pointcloud
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',

            #CAMERA INFO
            'camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

            #CAMAERA IMAGE
            'camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',  

            #GPS
            '/navsat1@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/navsat2@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',

                    ],
        remappings=[
        
            #rgbd
             # image
            ('rgbdcamera/image',
            '/camera/rgbd/image_raw'),


             ('rgbdcamera/camera_info',
            '/camera/rgbd/camera_info'),

            # Depth point cloud
            ('rgbdcamera/points',
            '/camera/rgbd/points'),

            # Depth image
            ('rgbdcamera/depth_image',
            '/camera/rgbd/depth_image'),

                 # push the LaserScan into /scan
            ('/lidar', '/scan'),
            # leave the pointcloud on /lidar/points
            ('/lidar/points', '/lidar/points'),
        ]


        
    )


    # 1) depth_image → /depth_scan
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'range_min': 0.5},        # adjust to your environment
            {'range_max': 20.0},
            {'scan_height': 10},
            {'output_frame_id': 'base_link'},
        ],
        remappings=[
            ('image', '/camera/rgbd/depth_image'),
            ('camera_info', '/camera/rgbd/camera_info'),
            ('scan', '/depth_scan'),
        ]
    )


    # Start Ignition Gazebo via gz_sim.launch.py
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

    #

    # Spawn the robot entity in Ignition
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
        '-entity', 'my_robot',
        '-topic', 'robot_description',
        # x, y, z
        '-x', '-10.0',
        '-y', '11',
        '-z', '0.1',
        # yaw (in radians). Positive rotates CCW, so –1.5708 is 90° right
        '--Y', '0'
    ],
     
    )

    # Load ros2_control + controllers
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers],
        output='screen'
    )

    # Controller spawners
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
   

    # TF relay from controller to /tf
    # topic_remapping = Node(
    #    package='topic_tools',
    #     executable='relay',
    #     name='tf_relay',
    #     arguments=['/ackermann_steering_controller/tf_odometry', '/tf'],
    #    output='screen'
    # )

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node_odom',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         PathJoinSubstitution([
    #             FindPackageShare('avone'),
    #             'config',
    #             'ekf.yaml'
    #         ])
    #     ],
    #     remappings=[('odometry/filtered', 'odometry/local')]
    # )

    # navsat_tf = Node(
    # package='robot_localization', executable='navsat_transform_node', name='navsat_transform',
    # parameters=[
    #             {'use_sim_time': use_sim_time},
    #             PathJoinSubstitution([
    #                 FindPackageShare('avone'),
    #                 'config',
    #                 'navsat_transform.yaml'
    #             ])
    #         ]
    # )

    # ekf_map = Node(
    #     package='robot_localization', executable='ekf_node', name='ekf_map',
    #     parameters=[
    #             {'use_sim_time': use_sim_time},
    #             PathJoinSubstitution([
    #                 FindPackageShare('avone'),
    #                 'config',
    #                 'map_ekf.yaml'
    #             ])
    #         ]
    # )

  

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

    return LaunchDescription([
        # sim time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation clock'
        ),
        # GPU env vars
        env_offload,
        env_vendor,
        # Robot state publisher
        rsp_node,
        # TF relay
        # topic_remapping,
        # Bridge
        ros_gz_bridge,
        # Gazebo sim
        gz_sim,
        # Spawn robot
        gz_spawn_entity,
        # ros2_control node
        ros2_control_node,

        depth_to_scan,
        
        #  ekf_node,
      
        # ekf_map,
       
        
        # Sequence controllers: joint_state -> ackermann -> brake
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
      
        # RViz
        rviz_node,

    ]
    )