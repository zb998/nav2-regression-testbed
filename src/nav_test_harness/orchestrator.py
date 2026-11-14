"""
给 SUT 的 /navigate_to_pose action 发目标，等它成功/失败/超时，顺路记录轨迹。这是"测试怎么驱动机器人干活"的核心。

@author: zbb
@date: 2026/09/13
@version: v1.0
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# Nav2 导航动作的消息类型
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from action_msgs.msg import GoalStatus


@dataclass
class TaskResult:
    status: str             # SUCCEEDED / ABORTED / TIMEOUT / SERVER_DOWN
    elapsed_s: float        # 耗时（秒）
    goal_position: tuple = (0.0, 0.0)
    final_position: tuple = (0.0, 0.0)
    arrival_error_m: Optional[float] = None
    # 完整轨迹点列表 [(x1,y1), (x2,y2)...]
    trajectory: list = field(default_factory=list)


class NavigationTaskRunner(Node):
    ACTION = '/navigate_to_pose'        # Nav2 标准的导航 Action 名称

    def __init__(self, scenario, node_name='nav_task_runner'):
        super().__init__(node_name)
        self._scenario = scenario
        # 创建动作客户端，准备给 Nav2 发导航指令
        self._client = ActionClient(self, NavigateToPose, self.ACTION)
        self._trajectory = []
        self._last_odom = None
        # 参数1：消息类型；参数2：话题名；参数3：回调函数，每收到一条消息就会自动调用；
        # 参数4：QoS 队列深度，表示最多缓存 10 条尚未处理的消息
        self._odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._trajectory.append((p.x, p.y))
        self._last_odom = msg

    def build_goal(self):
        """把 YAML 里的目标坐标翻译成 ROS 2 认识的 Goal 对象"""
        g = self._scenario.goal
        yaw = g.get('yaw', 0.0)     # 获取目标朝向（偏航角）
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(g['x'])
        goal.pose.pose.position.y = float(g['y'])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def send_goal(self, timeout_s=10.0):
        if not self._client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().error('NavigateToPose server not available')
            return False
        # 发送目标，获得一个 Future（未来完成的通知凭证）
        self._goal_future = self._client.send_goal_async(self.build_goal())
        deadline = time.time() + timeout_s
        # 显式创建一个 SingleThreadedExecutor 来管理 self
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        while not self._goal_future.done() and time.time() < deadline:
            # 最多等 0.1 秒，看有没有就绪的回调；有就执行一个，没有就返回。
            executor.spin_once(timeout_sec=0.1)
        executor.remove_node(self)
        if not self._goal_future.done():
            return False
        # 检查服务器是否接受了这个任务（比如目标点是否在地图外，可能会被拒绝）
        return bool(self._goal_future.result().accepted)

    def wait_for_result(self, timeout_s):
        t0 = time.time()
        # 从发指令的 Future 里取出任务句柄
        goal_handle = self._goal_future.result()
        result_future = goal_handle.get_result_async()
        deadline = t0 + timeout_s
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        while not result_future.done() and time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        executor.remove_node(self)
        elapsed = time.time() - t0
        goal = (self._scenario.goal['x'], self._scenario.goal['y'])
        if not result_future.done():
            status = 'TIMEOUT'
        else:
            result = result_future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                status = 'SUCCEEDED'
            elif result.status == GoalStatus.STATUS_ABORTED:
                status = 'ABORTED'
            else:
                status = f'STATUS_{result.status}'
        final = self._trajectory[-1] if self._trajectory else \
            (self._scenario.start['x'], self._scenario.start['y'])
        err = math.hypot(final[0] - goal[0], final[1] - goal[1])
        return TaskResult(status, elapsed, goal, final, err, self._trajectory)

    def execute(self, timeout_s=None):
        """对外暴露的唯一入口，按顺序执行：发指令 -> 等结果"""
        if not self.send_goal():
            return TaskResult('SERVER_DOWN', 0.0, trajectory=self._trajectory)
        budget = timeout_s or self._scenario.timeout_s
        return self.wait_for_result(budget)