"""
用纯 Python 代码在后台模拟一台 TurtleBot3 机器人

@author: zbb
@date: 2026/09/06
@version: v1.0
"""
from __future__ import annotations
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class MockSutNode(Node):
    def __init__(self):
        super().__init__('mock_sut')
        # 声明可调参数：行驶速度、到达容差、控制频率
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('arrival_tolerance', 0.1)
        self.declare_parameter('control_hz', 20.0)

        self._pose = [0.0, 0.0, 0.0]     # x, y, theta
        self._target = None
        self._last_cmd = Twist()         # 最新收到的速度指令
        self._avoid = False              # 是否触发避障

        # 发布“原始话题”，供故障注入器做手脚
        self._odom_pub = self.create_publisher(Odometry, '/odom_raw', 10)
        self._tf_pub = self.create_publisher(TFMessage, '/tf_raw', 10)
        self._scan_pub = self.create_publisher(LaserScan, '/scan_raw', 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # 订阅“加工后”话题（被注入器动过手脚的）
        self._cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

        # 创建 Action 服务端，接管 /navigate_to_pose 导航任务
        self._as = ActionServer(self, NavigateToPose, '/navigate_to_pose',
                                self._execute_goal)
        # 启动一个 20Hz 的定时器，作为机器人的“物理引擎”与“大脑”
        self.create_timer(1.0 / float(self.get_parameter('control_hz').value),
                          self._control_tick)

    # ---- 回调
    def _on_cmd_vel(self, msg):
        """收到外部（被注入过故障的）速度指令，更新内部指令状态"""
        self._last_cmd = msg

    def _on_scan(self, msg):
        """收到外部（被注入过噪声的）雷达数据，判断是否需要避障"""
        min_r = min(msg.ranges) if msg.ranges else 10.0
        self._avoid = min_r < 0.8      # 前方有“障碍”→ 减速

    def _execute_goal(self, goal_handle):
        """【Action 服务端】Jazzy 的写法：阻塞式移动到目标附近再返回结果。"""
        g = goal_handle.request.pose.pose
        self._target = [g.position.x, g.position.y]
        tol = float(self.get_parameter('arrival_tolerance').value)
        self.get_logger().info(f'nav goal accepted -> {self._target}')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._target = None
                return NavigateToPose.Result()
            # 计算当前距离目标的距离
            err = math.hypot(self._pose[0] - self._target[0],
                             self._pose[1] - self._target[1])
            if err <= tol:
                goal_handle.succeed()
                self._target = None
                return NavigateToPose.Result()
            # 没到的话，发布当前剩余距离反馈（Feedback），然后睡 50ms 继续下一轮
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = float(err)
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

        goal_handle.abort()
        return NavigateToPose.Result()

    # ---- 控制循环（20Hz）
    def _control_tick(self):
        dt = 1.0 / float(self.get_parameter('control_hz').value)

        # 1) 控制器：朝目标给期望速度（P 控制）
        tw = Twist()
        if self._target is not None:
            dx = self._target[0] - self._pose[0]
            dy = self._target[1] - self._pose[1]
            dist = math.hypot(dx, dy)
            goal_ang = math.atan2(dy, dx)
            ang_err = _wrap(goal_ang - self._pose[2])       # 角度差（归一化到 -pi ~ pi）
            # 线速度：距离越近越慢，最快不超过参数 speed
            tw.linear.x = max(0.0, min(dist, float(self.get_parameter('speed').value)))
            # 角速度：偏差越大转得越快，限制在 -1.0 到 1.0 之间
            tw.angular.z = max(-1.0, min(1.0, 1.2 * ang_err))
            if self._avoid:
                tw.linear.x *= 0.25   # 避障：减速前进
        # 把指令发给外部注入器（此时它还是干净的 _raw，注入器会拦截并加工）
        self._cmd_pub.publish(tw)      # -> 注入器 -> /cmd_vel

        # 2) 物理引擎（Plant）：积分“被注入过”的指令
        # 注意：这里使用的是 self._last_cmd（注入器返回的），而不是上面算出来的 tw。
        # 这完美模拟了真实世界中，控制器发出指令后，执行机构有可能没完美执行（比如被打滑/丢包了）
        self._pose[0] += self._last_cmd.linear.x * math.cos(self._pose[2]) * dt
        self._pose[1] += self._last_cmd.linear.x * math.sin(self._pose[2]) * dt
        self._pose[2] += self._last_cmd.angular.z * dt

        self._publish_state()

    def _publish_state(self):
        """把机器人的真实位置、TF、雷达数据发布出去（带 _raw 后缀）"""
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._pose[0]
        odom.pose.pose.position.y = self._pose[1]
        odom.pose.pose.orientation.z = math.sin(self._pose[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._pose[2] / 2.0)
        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self._pose[0]
        t.transform.translation.y = self._pose[1]
        t.transform.rotation.z = math.sin(self._pose[2] / 2.0)
        t.transform.rotation.w = math.cos(self._pose[2] / 2.0)
        tf = TFMessage()
        tf.transforms.append(t)
        self._tf_pub.publish(tf)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'base_link'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi / 180.0
        scan.range_min, scan.range_max = 0.1, 10.0
        scan.ranges = [10.0] * 360      # 模拟干净的空旷环境：360度全是 10 米
        self._scan_pub.publish(scan)


def _wrap(a):
    """辅助数学函数：把角度限制在 [-pi, pi] 之间，防止累加超出范围"""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main(args=None):
    rclpy.init()
    node = MockSutNode()
    # 必须使用多线程执行器，因为 _execute_goal 里有阻塞的 while 循环
    # 如果只有一个线程，控制循环 _control_tick 就会被卡死，机器人就动不了
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()