"""
度量节点（第一个 ROS 节点）。作用：订阅一组话题，统计频率，并采样 CPU/内存。

@author: zbb
@date: 2026/09/23
@version: v1.0
"""

from __future__ import annotations
import threading    # 用于线程锁，保证多线程下数据安全
import time
from dataclasses import dataclass, field
from typing import Optional
import rclpy
from rclpy.node import Node
# 这是一个 ROS 2 工具，能把字符串 'sensor_msgs/msg/LaserScan' 动态转换成真正的 Python 类
from rosidl_runtime_py.utilities import get_interface

# 尝试导入 psutil（用于读取系统 CPU/内存）
try:
    import psutil
except Exception:
    psutil = None

# 定义需要监控的 4 个核心话题及其对应的 ROS 2 消息类型
TOPIC_TYPES = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/odom': 'nav_msgs/msg/Odometry',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/tf': 'tf2_msgs/msg/TFMessage',
}


@dataclass
class MetricsSnapshot:
    """记录某个时间段内的各项指标"""
    rates: dict = field(default_factory=dict)   # 各个话题的频率，例如 {'/scan': 10.0}
    peak_cpu_pct: Optional[float] = None        # CPU 峰值占用率（%）
    peak_mem_mb: Optional[float] = None         # 内存峰值占用（MB）
    peak_mem_pct: Optional[float] = None        # 内存峰值占用（%）
    samples: int = 0                            # 采样次数（用来判断是否真的采到了数据）


class PerfMonitorNode(Node):
    def __init__(self, topics, window_s=5.0):
        super().__init__('perf_monitor')
        self._window_s = window_s       # 滑动窗口大小，默认看最近 5 秒的数据
        self._stamps = {}               # 字典，用于存储每个话题收到消息的时间戳列表
        # 创建线程锁，防止 ROS 回调（多线程）和主循环同时修改 _stamps 导致数据错乱
        self._lock = threading.Lock()
        # 创建一个 1 秒触发一次的定时器，用于清理过期数据
        self.create_timer(1.0, self._prune)
        for topic in topics:
            if topic not in TOPIC_TYPES:
                self.get_logger().warning(f'No known type for {topic}; skipping.')
                continue
            # 把字符串（如 'sensor_msgs/msg/LaserScan'）转成真正的消息类型对象
            typ = get_interface(TOPIC_TYPES[topic])
            self._stamps[topic] = []
            # 创建订阅者：收到消息就调用 self._make_cb(topic) 生成的闭包函数
            self.create_subscription(typ, topic, self._make_cb(topic), 10)
            

    def _make_cb(self, topic):
        """工厂函数：为每个话题生成一个专属的回调函数"""
        def cb(msg):
            # 获取当前 ROS 时间（纳秒转秒）
            now = self.get_clock().now().nanoseconds / 1e9
            with self._lock:    # 加锁保护共享数据
                self._stamps[topic].append(now)
        return cb

    def _prune(self):
        """定时清理：只保留最近 window_s 秒内的时间戳，防止内存无限增长"""
        now = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            for topic, stamps in self._stamps.items():
                self._stamps[topic] = [t for t in stamps if now - t <= self._window_s]

    def _rate(self, topic):
        """计算某个话题在最近 window_s 秒内的平均频率（Hz）"""
        now = self.get_clock().now().nanoseconds / 1e9
        with self._lock:
            stamps = [t for t in self._stamps.get(topic, []) if now - t <= self._window_s]
        if len(stamps) < 2:
            return 0.0
        return (len(stamps) - 1) / max(self._window_s, 1e-9)

    def _resources(self):
        """获取当前进程的 CPU 和内存占用情况"""
        if psutil is not None:
            try:
                p = psutil.Process()
                # cpu_percent(interval=None) 返回自上次调用以来的平均 CPU 使用率
                # rss / (1024 ** 2) 把字节转成 MB
                return p.cpu_percent(interval=None), \
                       p.memory_info().rss / (1024 ** 2), p.memory_percent()
            except Exception:
                pass
        return None, None, None

    def measure(self, duration_s):
        """
        核心测量函数：在指定的时长内持续采样 CPU 和内存，并计算各话题频率。
        duration_s: 测量时长（秒）
        """
        end = time.time() + duration_s
        peak = {'cpu': 0.0, 'mem_mb': 0.0, 'mem_pct': 0.0}
        n = 0       # 实际采样次数
        while time.time() < end:
            cpu, mem_mb, mem_pct = self._resources()
            if cpu is not None:
                peak['cpu'] = max(peak['cpu'], cpu)
                peak['mem_mb'] = max(peak['mem_mb'], mem_mb)
                peak['mem_pct'] = max(peak['mem_pct'], mem_pct)
                n += 1
            time.sleep(0.2)
        return MetricsSnapshot(
            rates={t: self._rate(t) for t in self._stamps},
            peak_cpu_pct=peak['cpu'] if n else None,
            peak_mem_mb=peak['mem_mb'] if n else None,
            peak_mem_pct=peak['mem_pct'] if n else None,
            samples=n,
        )


def main(args=None):
    """作为独立节点运行时的入口函数"""
    rclpy.init()
    node = PerfMonitorNode(['/scan', '/odom'])
    try:
        # 让节点一直运行，直到被 Ctrl+C 终止
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()