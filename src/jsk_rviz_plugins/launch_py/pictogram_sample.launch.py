#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch RViz GUI'
    )

    # Define nodes
    pictogram_sample_node = Node(
        package='jsk_rviz_plugins',
        executable='pictogram_all.py',
        name='pictogram_sample'
    )

    # RViz node with condition
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('jsk_rviz_plugins'),
        'config',
        'pictogram_sample.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        gui_arg,
        pictogram_sample_node,
        rviz_node
    ])