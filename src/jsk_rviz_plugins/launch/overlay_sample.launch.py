#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jsk_rviz_plugins',
            executable='overlay_sample',
            name='overlay_sample',
            output='screen',
            parameters=[
                {'rate': 100}
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '$(find-pkg-share jsk_rviz_plugins)/config/overlay_sample.rviz']
        )
    ])