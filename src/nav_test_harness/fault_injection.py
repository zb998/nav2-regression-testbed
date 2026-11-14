"""
做一个"中转节点"，把 SUT 的 ..._raw 话题转发给消费方，
测试可通过 SetParameters 服务在运行时切 drop / delay / noise / tf_jitter。不改任何 SUT 代码。

@author: zbb
@date: 2026/09/19
@version: v1.0
"""
from __future__ import annotations
import random
# 用于创建一个后台线程专门处理延迟消息
import threading
import time
# 双端队列，用于实现延迟队列
from collections import deque
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_interface
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


def injector_node_name(topic):
    """根据话题名生成节点名，避免冲突（如 '/scan' -> 'topic_injector_scan'）"""
    return 'topic_injector_' + topic.strip('/').replace('/', '_')


class TopicInjector(Node):
    """
    通/断/延迟/噪声 中继节点。
    所有行为都从 ROS 参数读取，可以在运行时通过 SetParameters 动态切换。
    """

    def __init__(self, node_name=None):
        super().__init__(node_name or injector_node_name('/scan'))
        # 声明所有需要的 ROS 参数（含默认值）
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('msg_type', 'sensor_msgs/msg/LaserScan')
        # 故障模式：直通（默认）/丢包/延迟/噪声
        self.declare_parameter('mode', 'passthrough')
        # 丢包率（0.0 ~ 1.0）
        self.declare_parameter('rate', 0.0)
        self.declare_parameter('delay_s', 0.0)
        self.declare_parameter('noise_std', 0.0)

        src = self.get_parameter('input_topic').value
        dst = self.get_parameter('output_topic').value
        mtype = self.get_parameter('msg_type').value
        typ = get_interface(mtype)
        # 创建订阅者（监听真实传感器）和发布者（转发给 Nav2）
        self._sub = self.create_subscription(typ, src, self._cb, 10)
        self._pub = self.create_publisher(typ, dst, 10)

        # 初始化延迟队列，并启动一个后台线程专门负责“闹钟放行”
        self._queue = deque()
        threading.Thread(target=self._delay_worker, daemon=True).start()

    def _cb(self, msg):
        """【核心回调】收到真实消息后，根据当前模式决定怎么注入故障"""
        mode = self.get_parameter('mode').value
        if mode == 'drop':
            # 【模式1：丢包】生成 0~1 的随机数，如果小于设定的丢包率，就直接 return（扔掉消息）
            if random.random() < float(self.get_parameter('rate').value):
                return
            self._pub.publish(msg)
        elif mode == 'delay':
            # 【模式2：延迟】不立即发，而是算出“未来释放的时间点”，塞进队列
            delay = float(self.get_parameter('delay_s').value)
            self._queue.append((time.time() + delay, msg))
        elif mode == 'noise':
            # 【模式3：噪声】先加噪声，再发出去
            std = float(self.get_parameter('noise_std').value)
            self._pub.publish(self._apply_noise(msg, std))
        else:
            # 【模式4：直通】什么都不做，原样转发
            self._pub.publish(msg)

    def _delay_worker(self):
        """【后台线程】不断检查队列，把到时间的消息发出去"""
        while rclpy.ok():
            now = time.time()
            # 如果队列不为空，且队头消息的释放时间到了
            while self._queue and self._queue[0][0] <= now:
                _, msg = self._queue.popleft()      # 弹出消息
                self._pub.publish(msg)              # 放行发送
            time.sleep(0.01)                        # 睡 10ms，避免 CPU 空转被拉满

    def _apply_noise(self, msg, std):
        """【加噪核心】根据消息类型，在数据里掺入高斯噪声"""
        # 获取当前时间，并转换成 ROS 2 的 Time 消息格式。
        now = self.get_clock().now().to_msg()
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            msg.header.stamp = now
        if isinstance(msg, LaserScan) and msg.ranges:
            # 雷达：给每个距离值加噪声，且不能小于 0（距离不能为负）
            msg.ranges = [max(0.0, v + random.gauss(0, std)) for v in msg.ranges]
        elif isinstance(msg, Twist):
            # 速度：线速度加噪声，角速度加一半强度的噪声（因为角速度更敏感）
            msg.linear.x += random.gauss(0, std)
            msg.angular.z += random.gauss(0, std * 0.5)
        elif isinstance(msg, Odometry):
            # 里程计：给 x, y 坐标加噪声
            msg.pose.pose.position.x += random.gauss(0, std)
            msg.pose.pose.position.y += random.gauss(0, std)
        elif isinstance(msg, TFMessage):
            # TF 坐标变换：给平移的 x, y 加噪声
            for t in msg.transforms:
                t.transform.translation.x += random.gauss(0, std)
                t.transform.translation.y += random.gauss(0, std)
        return msg


class TfJitterNode(Node):
    """【专用 TF 抖动节点】用均匀分布制造高频的坐标轴抖动"""
    def __init__(self, node_name=None):
        super().__init__(node_name or 'tf_jitter')
        self.declare_parameter('input_topic', '/tf_raw')
        self.declare_parameter('output_topic', '/tf')
        self.declare_parameter('jitter', 0.0)
        self._sub = self.create_subscription(TFMessage, self.get_parameter('input_topic').value,
                                             self._cb, 10)
        self._pub = self.create_publisher(TFMessage, self.get_parameter('output_topic').value, 10)

    def _cb(self, msg):
        """收到 TF 后，在 [-j, +j] 之间随机加减偏移量"""
        j = float(self.get_parameter('jitter').value)
        for t in msg.transforms:
            t.transform.translation.x += random.uniform(-j, j)
            t.transform.translation.y += random.uniform(-j, j)
        self._pub.publish(msg)


def main_injector(args=None):
    """作为独立节点运行时的入口：启动通用故障注入器"""
    rclpy.init()
    node = TopicInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_tf_jitter(args=None):
    """作为独立节点运行时的入口：启动 TF 抖动注入器"""
    rclpy.init()
    node = TfJitterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()