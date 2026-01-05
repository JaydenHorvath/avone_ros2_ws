# avone/launch/bringup.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ---- Launch args (optional; add more as you like) ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    can_interface = LaunchConfiguration('can_interface')
 

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
     
    ]

    # ---- Paths ----
    xacro_exe = PathJoinSubstitution([FindExecutable(name='xacro')])
    xacro_file = PathJoinSubstitution([FindPackageShare('avone'), 'description', 'robot.urdf.xacro'])
    robot_controllers = PathJoinSubstitution([FindPackageShare('avone'), 'config', 'ackermann_drive_controller.yaml'])
    rviz_config_path = os.path.expanduser('~/ros2_ws/src/avone/config/bringup.rviz')

    # ---- Build robot_description by running xacro ----
    # Pass any xacro parameters after the file path (examples shown)
    robot_description_cmd = Command([
        xacro_exe, ' ',
        xacro_file, ' ',
        'use_sim_time:=', use_sim_time, ' ',
        'can_interface:=', can_interface, ' ',
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_cmd, value_type=str)
    }

    # ---- Nodes ----
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, robot_controllers, {'update_rate': 50}],
        output='both',
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': use_sim_time}, robot_description],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_path],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
    )

    tf_odometry_relay = Node(
        package='topic_tools',
        executable='relay',
        name='tf_odometry_relay',
        arguments=['/ackermann_steering_controller/tf_odometry', '/tf'],
        output='screen'
    )

    # twiststamped_node = Node(
    #     package='avone',
    #     executable='twiststamped',
    #     name='twiststamped',
    #     output='screen'
    # )

    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     output='screen'
    # )

    cmd_filter_node = Node(
        package='avone_utils',
        executable='cmd_vel_filter',
        name='cmd_vel_filter',
        output='screen'


    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        parameters=[{
            'stamp': True,
            'axis_linear.x': 1,
            'scale_linear.x': 2.0,
            'axis_angular.yaw': 3,
            'scale_angular.yaw': 0.25,
            'enable_button': 4,
            'repeat_rate': 50.0
        }],
    )

    # static_map_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_odom_to_map',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )

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
        # twiststamped_node,
        # joy_node,
        teleop_twist_joy_node,
        # static_map_tf,
        cmd_filter_node,
    ]

    return LaunchDescription(nodes)
