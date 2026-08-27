"""Start waypoint navigation and prescription-driven UV treatment together."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    uv_share = get_package_share_directory('relay_control')
    navigation_share = get_package_share_directory('amiga_navigation')
    return LaunchDescription([
        DeclareLaunchArgument(
            'navigation_waypoints',
            default_value='',
            description='Navigation route CSV.',
        ),
        DeclareLaunchArgument(
            'uv_config',
            default_value=os.path.join(
                uv_share, 'config', 'uv_treatment.yaml'),
            description='UV treatment and variable-speed YAML.',
        ),
        DeclareLaunchArgument(
            'follower_config',
            default_value=os.path.join(
                navigation_share, 'config', 'waypoint_follower_params.yaml'),
            description='Copied waypoint follower YAML.',
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='pid_line',
            description='Navigation tracking controller.',
        ),
        DeclareLaunchArgument(
            'navigation_resume',
            default_value='ask',
            description='Waypoint follower resume mode: ask, yes, or no.',
        ),
        Node(
            package='relay_control',
            executable='uv_treatment_node',
            name='uv_treatment_node',
            output='screen',
            parameters=[LaunchConfiguration('uv_config')],
        ),
        Node(
            package='amiga_navigation',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[LaunchConfiguration('follower_config')],
            arguments=[
                '--waypoints', LaunchConfiguration('navigation_waypoints'),
                '--resume', LaunchConfiguration('navigation_resume'),
                '--controller', LaunchConfiguration('controller'),
            ],
        ),
    ])
