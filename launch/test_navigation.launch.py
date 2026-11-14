"""
只起 mock_sut，不带注入器，方便 ros2 topic echo /odom_raw 这种人工观察

@author: zbb
@date: 2026/10/05
@version: v1.0
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='nav_test_harness', executable='mock_sut', output='screen'),
    ])