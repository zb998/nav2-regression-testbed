"""
作用：等 action 就绪、远程切故障参数、跑一次导航。
把底层复杂的 ROS 2 通信包装成了极简的 Python 函数，让写测试用例的人能够专注于“怎么出题”，而不用管“怎么连接线缆”。

@author: zbb
@date: 2026/10/05
@version: v1.0
"""

from __future__ import annotations
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from nav2_msgs.action import NavigateToPose

from geometry_msgs.msg import PoseWithCovarianceStamped

# 映射表：知道哪个话题对应哪个注入器节点名
INJECTOR_NODES = {
    '/scan': 'scan_injector',
    '/odom': 'odom_injector',
    '/cmd_vel': 'cmd_vel_injector',
    '/tf': 'tf_jitter',
}


def wait_for_action_server(timeout=120.0):
    """【等待就绪】阻塞等待 Nav2 的导航服务器上线，防止测试跑得太快导致找不到目标"""
    # 创建一个带随机后缀的临时节点（防止多次调用时节点重名冲突）
    n = Node(f'waiter_{uuid.uuid4().hex[:6]}')
    client = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    end = time.time() + timeout
    ok = False
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        if client.server_is_ready():
            ok = True
            break
    n.destroy_node()        # 用完销毁，释放资源
    return ok

def set_initial_pose(x: float = 1.6, y: float = 0.0, yaw: float = 0.0) -> None:
    """Publish /initialpose so AMCL can localize (needed for real Gazebo SUT)."""
    n = Node(f'initial_pose_{uuid.uuid4().hex[:6]}')
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    time.sleep(0.5)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = n.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.w = 1.0
    msg.pose.covariance[0] = 0.25
    msg.pose.covariance[7] = 0.25
    msg.pose.covariance[35] = 0.068
    for _ in range(3):
        pub.publish(msg)
        rclpy.spin_once(n, timeout_sec=0.1)
        time.sleep(0.2)
    n.destroy_node()


def set_param(node_name, name, value):
    """【改参数】远程修改指定节点的参数（比如把丢包率从 0 改到 0.5）"""
    n = Node(f'param_{uuid.uuid4().hex[:6]}')
    # 创建客户端，连接目标节点的 set_parameters 服务
    cli = n.create_client(SetParameters, f'/{node_name}/set_parameters')
    if not cli.wait_for_service(timeout_sec=5.0):
        n.destroy_node()
        raise RuntimeError(f'SetParameters service of {node_name} not available')
    # 构造请求，把 Python 值转换成 ROS 参数消息
    req = SetParameters.Request()
    req.parameters = [Parameter(name, value=value).to_parameter_msg()]
    # 发起异步调用
    fut = cli.call_async(req)
    # 原地打转，等服务器返回结果
    while not fut.done() and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node()
    # 检查服务端是否回复“设置成功”
    if fut.done() and fut.result():
        return fut.result().results[0].successful
    return False


def reset_faults():
    for topic, node in INJECTOR_NODES.items():
        if topic == '/tf':
            set_param(node, 'jitter', 0.0)
        else:
            set_param(node, 'mode', 'passthrough')


def apply_fault(fault):
    """【下发故障】接收一个 FaultSpec 对象，把故障参数精准下发到对应的注入器"""
    if fault.kind == 'tf_jitter':
        return set_param(INJECTOR_NODES['/tf'], 'jitter', fault.rate)
    node = INJECTOR_NODES.get(fault.topic)
    if node is None:
        return False
    for key, val in fault.as_params().items():
        set_param(node, key, val)
    return True


def spin_in_background(node):
    """【后台自旋】开个后台线程偷偷处理 ROS 消息，让主线程可以安心 sleep 或做耗时计算"""
    import threading
    stop = threading.Event()

    def _spin():
        # 只要没有收到停止信号，且 ROS 还在运行，就不停地 spin
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    # 设置为 daemon 守护线程，主程序退出时它也会跟着退出
    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return stop, thread


def stop_background_spin(stop, thread):
    """【停后台自旋】发送停止信号，并等待线程安全退出"""
    stop.set()                  # 发出 Event 信号
    thread.join(timeout=3)      # 最多等 3 秒让线程收尾


def run_nav(scenario):
    """【一键跑导航】封装了 orchestrator，传给测试用例直接拿到最终结果"""
    from nav_test_harness.orchestrator import NavigationTaskRunner
    runner = NavigationTaskRunner(scenario)
    try:
        return runner.execute()
    finally:
        runner.destroy_node()       # 无论成功失败，最后一定要销毁节点