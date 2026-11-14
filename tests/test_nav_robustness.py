"""
故意给机器人制造各种麻烦（丢包、延迟、噪声、坐标抖动），看看它还能不能成功到达终点

@author: zbb
@date: 2026/10/10
@version: v1.0
"""

import math
import time
import uuid
import pytest
import rclpy
from rclpy.node import Node
from nav_test_harness import assertions, metrics
# 引入测试辅助工具：跑导航、下放故障、重置故障、后台自旋、停后台
from helpers import run_nav, apply_fault, reset_faults, spin_in_background, stop_background_spin


def _monitor_run(scenario):
    """【通用陪跑裁判】一边跑导航，一边监控系统性能"""
    # 监控 /scan, /odom, /cmd_vel 这三个话题
    mon = metrics.PerfMonitorNode(['/scan', '/odom', '/cmd_vel'], window_s=5.0)
    stop, thr = spin_in_background(mon)     # 后台自旋，保证监控节点能持续收数据
    try:
        result = run_nav(scenario)          # 主线程去跑导航
    finally:
        stop_background_spin(stop, thr)
    snap = mon.measure(3.0)                 # 收集跑完前最后 3 秒的数据快照
    mon.destroy_node()
    return result, snap


@pytest.mark.parametrize('scenario', ['cmd_vel_drop'], indirect=True)
def test_robust_cmd_vel_drop(ros, sut, scenario):
    reset_faults()
    apply_fault(scenario.faults[0])
    result, snap = _monitor_run(scenario)
    goal = (scenario.goal['x'], scenario.goal['y'])
    th = scenario.effective_thresholds()
    results = [
        assertions.assert_arrival(result.final_position, goal, scenario.arrival_tolerance_m),
        assertions.assert_within_time(result.elapsed_s, th['budget_s']),
        assertions.assert_scan_rate_ok(snap.rates.get('/scan'), th['min_scan_hz']),
    ]
    assert result.status == 'SUCCEEDED', f'nav status = {result.status}'
    assert all(r.passed for r in results), assertions.summarize(results)
    # 注入有效性：mock 控制器约 20Hz 发 /cmd_vel，drop 后应明显下降
    assert snap.rates.get('/cmd_vel', 20.0) < 15.0, \
        f'cmd_vel rate {snap.rates.get("/cmd_vel")}Hz -> drop injection had no effect'


@pytest.mark.parametrize('scenario', ['cmd_vel_delay'], indirect=True)
def test_robust_cmd_vel_delay(ros, sut, scenario):
    reset_faults()
    apply_fault(scenario.faults[0])
    result, _ = _monitor_run(scenario)      # 这里不需要性能数据，用 _ 占位丢弃
    goal = (scenario.goal['x'], scenario.goal['y'])
    th = scenario.effective_thresholds()
    results = [
        assertions.assert_arrival(result.final_position, goal, scenario.arrival_tolerance_m),
        assertions.assert_within_time(result.elapsed_s, th['budget_s']),
    ]
    assert result.status == 'SUCCEEDED'
    assert all(r.passed for r in results), assertions.summarize(results)


@pytest.mark.parametrize('scenario', ['lidar_drop'], indirect=True)
def test_robust_lidar_drop(ros, sut, scenario):
    reset_faults()
    apply_fault(scenario.faults[0])
    result, snap = _monitor_run(scenario)
    goal = (scenario.goal['x'], scenario.goal['y'])
    th = scenario.effective_thresholds()
    results = [
        assertions.assert_arrival(result.final_position, goal, scenario.arrival_tolerance_m),
        assertions.assert_within_time(result.elapsed_s, th['budget_s']),
    ]
    assert result.status == 'SUCCEEDED'
    assert all(r.passed for r in results), assertions.summarize(results)
    assert snap.rates.get('/scan', 20.0) < 15.0, \
        f'scan rate {snap.rates.get("/scan")}Hz -> drop injection had no effect'


@pytest.mark.parametrize('scenario', ['lidar_noise'], indirect=True)
def test_robust_lidar_noise(ros, sut, scenario):
    reset_faults()
    apply_fault(scenario.faults[0])
    result, _ = _monitor_run(scenario)
    goal = (scenario.goal['x'], scenario.goal['y'])
    th = scenario.effective_thresholds()
    results = [
        assertions.assert_arrival(result.final_position, goal, scenario.arrival_tolerance_m),
        assertions.assert_within_time(result.elapsed_s, th['budget_s']),
    ]
    assert result.status == 'SUCCEEDED'
    assert all(r.passed for r in results), assertions.summarize(results)


@pytest.mark.parametrize('scenario', ['odom_noise'], indirect=True)
def test_robust_odom_drift(ros, sut, scenario):
    reset_faults()
    apply_fault(scenario.faults[0])
    result, _ = _monitor_run(scenario)
    goal = (scenario.goal['x'], scenario.goal['y'])
    results = [assertions.assert_odom_deviation(result.trajectory, goal, max_dev_m=0.5)]
    assert result.status == 'SUCCEEDED', f'nav status = {result.status}'
    assert all(r.passed for r in results), assertions.summarize(results)


@pytest.mark.parametrize('scenario', ['tf_jitter'], indirect=True)
def test_robust_tf_jitter(ros, sut, scenario):
    reset_faults()
    apply_fault(scenario.faults[0])
    # 这里没调用 _monitor_run，因为抖动不需要性能监控
    result = run_nav(scenario)
    # 【特殊采样】手动去采集 3 秒内的 TF 数据，算出最大跳跃距离
    jitter_max = _sample_tf_jitter(duration_s=3.0)
    goal = (scenario.goal['x'], scenario.goal['y'])
    results = [
        assertions.assert_arrival(result.final_position, goal, scenario.arrival_tolerance_m),
        assertions.assert_tf_jitter_detected(jitter_max, limit_m=0.3),      # 抖动不能超过 0.3 米
    ]
    assert result.status == 'SUCCEEDED'
    assert all(r.passed for r in results), assertions.summarize(results)


def _sample_tf_jitter(duration_s):
    """辅助函数：计算 TF 抖动的最大跳跃距离"""
    from tf2_msgs.msg import TFMessage
    node = Node(f'tf_sampler_{uuid.uuid4().hex[:6]}')
    samples = []
    # 创建一个临时订阅者，把收到的 TF 消息全塞进 samples 列表里
    node.create_subscription(TFMessage, '/tf', lambda m: samples.append(m), 10)
    end = time.time() + duration_s
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    pts = []
    for m in samples:
        for t in m.transforms:
            if t.child_frame_id == 'base_link':
                pts.append((t.transform.translation.x, t.transform.translation.y))
    if len(pts) < 2:
        return None
    return max(math.hypot(pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1])
               for i in range(1, len(pts)))