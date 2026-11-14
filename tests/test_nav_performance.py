"""
性能测试：主题频率和主机资源是否满足阈值。

@author: zbb
@date: 2026/10/15
@version: v1.0
"""

import pytest
from nav_test_harness import assertions, metrics
from helpers import run_nav, spin_in_background, stop_background_spin


@pytest.mark.parametrize('scenario', ['simple_line'], indirect=True)
def test_perf_scan_rate_and_resources(ros, sut, scenario):
    mon = metrics.PerfMonitorNode(['/scan', '/odom'], window_s=5.0)
    stop, thr = spin_in_background(mon)
    try:
        result = run_nav(scenario)
    finally:
        stop_background_spin(stop, thr)
    snap = mon.measure(3.0)
    mon.destroy_node()
    th = scenario.effective_thresholds()
    results = [
        assertions.assert_scan_rate_ok(snap.rates.get('/scan'), th['min_scan_hz']),
        assertions.assert_resource_within(snap, th),
        assertions.assert_within_time(result.elapsed_s, th['budget_s']),
    ]
    assert all(r.passed for r in results), assertions.summarize(results)


@pytest.mark.parametrize('scenario', ['simple_line'], indirect=True)
def test_perf_nav_duration_recorded(ros, sut, scenario):
    result = run_nav(scenario)
    th = scenario.effective_thresholds()
    assert result.status == 'SUCCEEDED'
    assert 0 < result.elapsed_s <= th['budget_s'], \
        f'elapsed {result.elapsed_s}s outside (0, {th["budget_s"]}]'