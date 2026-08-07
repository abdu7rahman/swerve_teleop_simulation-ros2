"""Spawn the swerve base in Gazebo Sim with ros2_control and teleop.

ROS 2 port of swerve_description/launch/gazebo.launch and controller.launch,
which had to be run separately in ROS 1.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('swerve_description')
    bringup_share = get_package_share_directory('swerve_bringup')

    xacro_file = os.path.join(description_share, 'urdf', 'swerve.xacro')
    params = os.path.join(bringup_share, 'config', 'swerve_params.yaml')

    world = LaunchConfiguration('world')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware:=gazebo']), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # ROS 1 used gazebo_ros spawn_model reading the robot_description param.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'swerve', '-z', '0.2'],
    )

    # Controllers have to be spawned after the entity exists in the world,
    # otherwise the controller_manager the gz plugin starts is not yet up.
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['steering_position_controller'],
    )

    wheel_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_velocity_controller'],
    )

    kinematics = Node(
        package='swerve_bringup',
        executable='swerve_kinematics',
        output='screen',
        parameters=[params, {'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='empty.sdf',
            description='Gazebo Sim world file.'),

        gz_sim,
        bridge,
        robot_state_publisher,
        spawn,

        RegisterEventHandler(OnProcessExit(
            target_action=spawn,
            on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[steering_controller, wheel_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=wheel_controller,
            on_exit=[kinematics])),
    ])
