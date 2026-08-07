"""View the swerve base in rviz2 with joint sliders.

ROS 2 port of swerve_description/launch/display.launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('swerve_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'swerve.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    gui = LaunchConfiguration('gui')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' hardware:=mock']), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),

        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher_gui',
             executable='joint_state_publisher_gui',
             condition=IfCondition(gui)),

        Node(package='joint_state_publisher',
             executable='joint_state_publisher',
             condition=UnlessCondition(gui)),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', rviz_config]),
    ])
