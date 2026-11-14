"""
接收测试跑出来的各种测量数据（比如位置、耗时、CPU占用），然后用一套标准去评判它们，最终给出 PASS（通过）或 FAIL（失败）的结论。

@author: zbb
@date: 2026/09/01
@version: v1.0
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence


@dataclass
class AssertionResult:
    name: str                # 断言的名字（比如 'arrival'）
    passed: bool             # 结果：True(及格) 或 False(不及格)
    detail: str              # 详细说明（比如：误差 0.15m，要求 0.2m 以内）
    quality_property: str    # 质量属性标签（功能、鲁棒性、性能）


def _res(name, passed, detail, qp):
    """内部辅助函数：快速生成一个 AssertionResult 对象，避免每次都写长长的类名"""
    return AssertionResult(name, passed, detail, qp)


# ---- 功能正确性
def assert_arrival(pos, goal, tolerance_m, label='arrival'):
    # math.hypot 计算两点之间的欧几里得距离：√((x2-x1)² + (y2-y1)²)
    err = math.hypot(pos[0] - goal[0], pos[1] - goal[1])
    return _res(label, err <= tolerance_m,
                f'final_error={err:.3f}m (tolerance {tolerance_m}m)',
                'functional-correctness')


def assert_within_time(elapsed_s, budget_s, label='time-budget'):
    return _res(label, elapsed_s <= budget_s,
                f'elapsed={elapsed_s:.2f}s (budget {budget_s:.1f}s)',
                'performance')


# ---- 鲁棒性
def assert_no_collision(min_scan_range_m, label='no-collision'):
    # 如果 min_scan_range_m 是 None（没数据），或者它大于 0.10米，就算安全
    ok = min_scan_range_m is None or min_scan_range_m > 0.10
    # 下面的三元表达式用于生成说明文字：如果有数据，显示距离；没数据，显示 'no scan data'
    return _res(label, ok,
                f'min_forward_range={min_scan_range_m:.3f}m' if min_scan_range_m is not None
                else 'no scan data', 'robustness')


def assert_odom_deviation(actual_path, goal, max_dev_m, label='odom-stability'):
    """噪声下记录轨迹离“起点到目标直线”的最大偏移，过大即定位漂移。"""
    if not actual_path:
        return _res(label, False, 'no odom samples recorded', 'robustness')
    # 获取起点 (sx, sy) 和终点 (gx, gy)
    sx, sy = actual_path[0]
    gx, gy = goal
    worst = 0.0    # 记录最大偏移量
    for (x, y) in actual_path:
        # 计算该点离“起点到终点连成的线段”的垂直距离
        d = _point_seg_dist(x, y, sx, sy, gx, gy)
        worst = max(worst, d)
    return _res(label, worst <= max_dev_m,
                f'max_deviation={worst:.3f}m (limit {max_dev_m}m)', 'robustness')


def _point_seg_dist(px, py, ax, ay, bx, by):
    """【数学工具】计算点 (px, py) 到线段 AB 的最短距离"""
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    # 利用向量点乘计算投影比例 t，并限制在 [0, 1] 之间（保证落在线段上而不是无限长的直线上）
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    # 计算点到线段上投影点的距离
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def assert_tf_jitter_detected(jitter_max, limit_m, label='tf-jitter-limit'):
    """判断 TF（坐标变换）的抖动是否在容许范围内"""
    if jitter_max is None:
        return _res(label, True, 'no tf sampled', 'robustness')
    return _res(label, jitter_max <= limit_m,
                f'tf_translation_jitter={jitter_max:.3f}m (limit {limit_m}m)',
                'robustness')


def assert_cmd_vel_degraded(base_rate_hz, injected_rate_hz, drop_expected,
                            label='injection-effective'):
    """测试故障注入器是否真的起作用了"""
    # 逻辑：如果没有预期丢包 (not drop_expected)，则通过；
    # 如果预期丢包了，则注入后的频率(injected) 必须明显小于 基础频率(base) 的 90%，才证明故障注入生效了
    ok = not drop_expected or injected_rate_hz < base_rate_hz * 0.9
    return _res(label, ok,
                f'cmd_vel rate {base_rate_hz:.1f}Hz -> {injected_rate_hz:.1f}Hz',
                'robustness')


# ---- 性能
def assert_scan_rate_ok(rate_hz, min_hz, label='scan-rate'):
    """判断雷达扫描频率是否达标"""
    # 频率不能为空，且必须大于等于设定的最小频率 (min_hz)
    ok = rate_hz is not None and rate_hz >= min_hz
    return _res(label, ok,
                f'scan rate={rate_hz:.2f}Hz (min {min_hz}Hz)' if rate_hz is not None
                else 'no scan messages', 'performance')


def assert_resource_within(snapshot, thresholds, label='resources'):
    """判断 CPU 和 内存 的峰值是否超标"""
    cpu = snapshot.peak_cpu_pct     # 假设 snapshot 对象里存了 CPU 峰值
    mem = snapshot.peak_mem_pct
    ok = (cpu is None or cpu <= thresholds.get('max_cpu_pct', 100.0)) and \
         (mem is None or mem <= thresholds.get('max_mem_pct', 100.0))
    return _res(label, ok,
                f'peak_cpu={cpu}%, peak_mem={mem}%',
                'performance')


def summarize(results):
    """将所有零散的 AssertionResult 汇总成一段纯文本报告"""
    lines = []
    for r in results:
        mark = 'PASS' if r.passed else 'FAIL'
        lines.append(f'  [{mark}] {r.name} ({r.quality_property}): {r.detail}')
    failed = [r for r in results if not r.passed]
    lines.insert(0, f'{len(results) - len(failed)}/{len(results)} assertions passed')
    return '\n'.join(lines)