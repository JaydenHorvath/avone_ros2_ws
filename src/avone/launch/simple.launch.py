from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Bridge between Ignition and ROS 2
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/my_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/model/my_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            ('/model/my_robot/tf', '/tf'),
            ('/model/my_robot/odometry', '/odom'),
        ],
        output='screen'
    )

    # RViz node (delayed until robot is spawned)
    rviz_node = TimerAction(
        period=8.0,  # seconds, should be after robot spawn (5.0s)
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    robot_controllers = '/home/jay/ros2_ws/src/avone/config/ackermann_drive_controller.yaml'


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )

    return LaunchDescription([
        # Set environment variables for NVIDIA GPU offload
        SetEnvironmentVariable(name='__NV_PRIME_RENDER_OFFLOAD', value='1'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),

        # Start Ignition Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Spawn robot after delay
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_ign_gazebo',
                    executable='create',
                    arguments=[
                        '-name', 'my_robot',
                        '-topic', 'robot_description',
                        '-x', '0', '-y', '0', '-z', '0.5'
                    ],
                    output='screen'
                )
            ]
        ),

        # Start the bridge
        gz_ros2_bridge,

        # Start RViz after everything else is ready
        rviz_node
    ])
