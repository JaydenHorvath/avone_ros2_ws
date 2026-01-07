#!/usr/bin/env python3

# AV.ONE Simulation Bringup
# File: avone_sim.launch.py
# Launch: ros2 launch avone avone_sim.launch.py

# Purpose:
#   - Launch the Gazebo simulator world
#   - Spawn the AV.ONE robot into the world
#   - Start ros2_control (GazeboSimSystem) + controllers (Ackermann + joint_state_broadcaster)
#   - Bridge Gazebo topics into ROS 2 (clock, TF, sensors, navsat, camera, lidar)
#   - Start robot_localization (EKF + navsat_transform) for simulated localisation
#   - Start basic teleop (joy + teleop_twist_joy + cmd_vel_filter)
#   - Optionally start RViz after the sim is up

# Notes (important for this stack):
#   - robot.urdf.xacro must include ros2_control.xacro (GazeboSimSystem) not ros2_control_hardware.xacro (comment out which one isnt being used)
#   - use_sim_time must be true everywhere (robot_state_publisher, bridge, EKF, RViz)
#   - In robot_core.xacro, mesh paths may need to be hard-coded file:///... for Gazebo on a given machine
#     (package:// paths are best for ROS usage but do not work with GAZEBO)
#   - This launch file sets GZ_SIM_RESOURCE_PATH and IGN_GAZEBO_RESOURCE_PATH to the package models folder.


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
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    # Use /clock if running in Gazebo
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # Paths to your YAMLs in avone/config
    avone_ekf_yaml = PathJoinSubstitution(
        [FindPackageShare("avone_localisation"), "config", "avone_ekf.yaml"]
    )

    models_path = PathJoinSubstitution([FindPackageShare("avone"), "worlds", "models"])

    set_ign_models = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH", value=models_path
    )

    set_gz_models = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", value=models_path
    )

    # OPTIONAL: Ensure gazebo runs on nvidia for GPU offload
    # env_offload = SetEnvironmentVariable(
    #     name='__NV_PRIME_RENDER_OFFLOAD',
    #     value='1'
    # )
    # env_vendor = SetEnvironmentVariable(
    #     name='__GLX_VENDOR_LIBRARY_NAME',
    #     value='nvidia'
    # )

    # Robot description via xacro
    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare("avone"), "description", "robot.urdf.xacro"]
                    ),
                ]
            ),
            value_type=str,
        ),
        "use_sim_time": use_sim_time,
    }

    # Controllers configuration
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("avone"), "config", "ackermann_drive_controller.yaml"]
    )

    # -------------------------------------------------------------------------
    # robot_localization Nodes
    # -------------------------------------------------------------------------

    # Simulator imu remap
    imu_to_imu_gps = Node(
        package="topic_tools",
        executable="relay",
        name="relay_imu_to_imu_gps",
        arguments=["/imu", "/imu_gps"],
        output="screen",
    )

    # Simulator imu remap
    imu_to_imu_data = Node(
        package="topic_tools",
        executable="relay",
        name="relay_imu_to_imu_data",
        arguments=["/imu", "/imu/data"],
        output="screen",
    )

    # Simplified Single EKF for simulator use
    ekf_avone_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, avone_ekf_yaml],
        remappings=[
            ("odometry/filtered", "/odometry/avone"),
        ],
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"frequency": 30.0, "publish_filtered_gps": True},
            {"broadcast_utm_transform": False},
        ],
        remappings=[
            ("gps/fix", "/fix"),
            ("odometry/filtered", "/odometry/avone"),
            ("odometry/gps", "/odometry/gps1"),
            ("imu", "/imu"),
        ],
    )
    # Static map->odom for sim convenience, as no dual ekf
    static_map_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_map",
        output="screen",
        arguments=[
            "0",
            "0",
            "0",  # x y z
            "0",
            "0",
            "0",  # roll pitch yaw
            "map",
            "odom",  # parent_frame child_frame
        ],
    )

    # Robot State Publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ============================================================
    # Gazebo sim bringup + spawn entity
    # ============================================================

    # ROS <-> Gazebo topic bridge (clock, tf, sensors)
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/my_robot/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/model/my_robot/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "rgbdcamera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "rgbdcamera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "rgbdcamera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "rgbdcamera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/navsat1@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            "/navsat2@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
        ],
        remappings=[
            ("rgbdcamera/image", "/camera/rgbd/image_raw"),
            ("rgbdcamera/camera_info", "/camera/rgbd/camera_info"),
            ("rgbdcamera/points", "/camera/rgbd/points"),
            ("rgbdcamera/depth_image", "/camera/rgbd/depth_image"),
            ("/lidar", "/scan"),
            ("/lidar/points", "/lidar/points"),
            ("/navsat1", "/fix"),
        ],
    )

    # Gazebo world selction, options in "worlds" folder
    world_file = PathJoinSubstitution(
        [FindPackageShare("avone"), "worlds", "smalltrack.world"]
    )

    # Gazebo Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r -v1 "), world_file]
        }.items(),
    )

    #  Spawn Avone in Gazebo, with respective x,y,z position
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-entity",
            "my_robot",
            "-topic",
            "robot_description",
            # '-x', '0',
            # '-y', '0',
            # '-z', '0',
            # '--Y', '0'
            # # small track
            "-x",
            "-10.0",
            "-y",
            "11",
            "-z",
            "0.1",
            "--Y",
            "0",
            # accel
            # '-x', '25.2118',
            # '-y', '-0.2167',
            # '-z', '0.1',
            # '--Y', '3.14'
        ],
    )

    # ============================================================
    # ros2_control (GazeboSimSystem) + controller spawners
    # ============================================================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
        output="screen",
    )

    # RViz (delayed until simulation is ready)
    rviz_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    # Cmd Filter, filtering cmd_vel before being passed to control
    cmd_filter_node = Node(
        package="avone_utils",
        executable="cmd_vel_filter",
        name="cmd_vel_filter",
        output="screen",
    )

    # Joystick control
    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen"
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="screen",
        parameters=[
            {
                "stamp": True,
                "axis_linear.x": 1,
                "scale_linear.x": 2.0,
                "axis_angular.yaw": 3,
                "scale_angular.yaw": 0.5,
                "enable_button": 4,
                "repeat_rate": 50.0,
            }
        ],
    )

    # Launch Description
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock",
            ),
            set_ign_models,
            set_gz_models,
            # ----------------------------------------------------
            # 2) GPU environment variables
            # ----------------------------------------------------
            # env_offload,
            # env_vendor,
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
            # 9) Sequence controller spawners
            # ----------------------------------------------------
            RegisterEventHandler(
                OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[ackermann_spawner],
                )
            ),
            # ----------------------------------------------------
            # 10) robot_localization Nodes
            # ----------------------------------------------------
            static_map_tf,
            ekf_avone_node,
            navsat_transform_node,
            imu_to_imu_gps,
            imu_to_imu_data,
            # ----------------------------------------------------
            # 11) RViz (delayed)
            # ----------------------------------------------------
            rviz_node,
            # ----------------------------------------------------
            # 12) Teleop: joy_node & teleop_twist_joy_node
            # ----------------------------------------------------
            joy_node,
            cmd_filter_node,
            teleop_twist_joy_node,
        ]
    )
