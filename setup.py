"""
Python 包的安装包清单 + 注册可执行节点

@author: zbb
@date: 2026/08/26
@version: v1.0
"""

from setuptools import find_packages, setup

package_name = 'nav_test_harness'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup_sut_mock.launch.py',
            'launch/bringup_sut_real.launch.py',
            'launch/test_navigation.launch.py',
        ]),
        ('share/' + package_name + '/scenarios', [
            'scenarios/simple_line.yaml',
            'scenarios/corner_turn.yaml',
            'scenarios/edge_close_target.yaml',
            'scenarios/cmd_vel_drop.yaml',
            'scenarios/cmd_vel_delay.yaml',
            'scenarios/lidar_drop.yaml',
            'scenarios/lidar_noise.yaml',
            'scenarios/odom_noise.yaml',
            'scenarios/tf_jitter.yaml',
        ]),
        ('share/' + package_name + '/config', [
            'config/perf_thresholds.yaml',
            'config/nav2_params.yaml',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/simple_world.sdf',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zbb',
    maintainer_email='you@example.com',
    description='Nav2 navigation regression testbed for ROS2 + Gazebo.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mock_sut = nav_test_harness.mock_sut:main',
            'topic_injector = nav_test_harness.fault_injection:main_injector',
            'tf_jitter = nav_test_harness.fault_injection:main_tf_jitter',
            'perf_monitor = nav_test_harness.metrics:main',
        ],
    },
)
