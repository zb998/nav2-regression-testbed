"""
测试演示：启动模拟SUT并断言一个主题存在

@author: zbb
@date: 2026/10/18
@version: v1.0
"""
import time
import unittest
import launch
import launch_testing
import launch_testing.markers
import launch_testing.actions
import pytest
import rclpy
from launch_ros.actions import Node
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        Node(package='nav_test_harness', executable='mock_sut', output='screen'),
        launch_testing.actions.ReadyToTest(),
    ])


@launch_testing.markers.keep_alive
class ActiveTests(unittest.TestCase):
    def test_scan_raw_topic_publishes(self):
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node('lt_scan_check')
        received = []
        node.create_subscription(LaserScan, '/scan_raw',
                                 lambda m: received.append(m), 10)
        end = time.time() + 10.0
        while time.time() < end and not received:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.assertTrue(received, 'no /scan_raw messages were received')