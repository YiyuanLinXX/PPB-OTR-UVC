"""Compatibility launch alias for the UV treatment controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('relay_control'),
        'config',
        'uv_treatment.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Absolute path to the UV treatment YAML file.',
        ),
        Node(
            package='relay_control',
            executable='uv_treatment_node',
            name='uv_treatment_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
