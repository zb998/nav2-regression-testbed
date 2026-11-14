"""
一键拉起“假机器人（Mock SUT）”以及配套的 4 个“故障注入器”，全程不需要启动臃肿的 Gazebo 仿真。
测试切参数即可注入故障。

@author: zbb
@date: 2026/10/03
@version: v1.0
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义一个话题注入器配置列表
# 每个元组包含：(节点名称, 输入话题/源话题, 输出话题/目标话题, 消息类型)
INJECTORS = [
    ('scan_injector', '/scan_raw', '/scan', 'sensor_msgs/msg/LaserScan'),
    ('odom_injector', '/odom_raw', '/odom', 'nav_msgs/msg/Odometry'),
    ('cmd_vel_injector', '/cmd_vel_raw', '/cmd_vel', 'geometry_msgs/msg/Twist'),
]


def generate_launch_description():
    # 初始化节点列表，首先添加核心的模拟被测系统 (MOCK SUT) 节点
    # package: 指定功能包名称为 'nav_test_harness'
    # executable: 指定要运行的可执行文件 'mock_sut'
    # output: 设置输出方式为 'screen'，即日志会打印到屏幕上
    nodes = [Node(package='nav_test_harness', executable='mock_sut', output='screen')]
    # 遍历 INJECTORS 列表，为每个配置创建一个话题注入节点
    for name, src, dst, mtype in INJECTORS:
        nodes.append(Node(
            package='nav_test_harness',
            executable='topic_injector',        # 可执行文件：话题注入器
            output='log',
            name=name,                          # 节点名称（来自列表配置）
            parameters=[{'input_topic': src, 'output_topic': dst, 'msg_type': mtype}],
        ))
    
    # 添加一个 TF 坐标变换抖动模拟节点，此处 jitter 设为 0.0 表示无抖动
    nodes.append(Node(
        package='nav_test_harness',
        executable='tf_jitter',
        output='log',
        name='tf_jitter',
        parameters=[{'input_topic': '/tf_raw', 'output_topic': '/tf', 'jitter': 0.0}],
    ))
    # 返回包含所有节点的启动描述对象
    return LaunchDescription(nodes)