"""
检查在不加任何故障（不丢包、不延迟、不加噪）的正常情况下，机器人能不能老老实实走到终点。

@author: zbb
@date: 2026/10/08
@version: v1.0
"""

import pytest
from nav_test_harness import assertions, report
from helpers import run_nav


# 测试用例 1：正常路径导航到达（包含直线和拐弯两个场景）
# indirect=True 表示不直接传字符串，而是把字符串交给 conftest.py 里的 scenario fixture 去加载 YAML
@pytest.mark.parametrize('scenario', ['simple_line', 'corner_turn'], indirect=True)
def test_functional_arrival(ros, sut, scenario):
    # 执行导航任务（底层会自动发送目标，并阻塞等待机器人走到终点）
    result = run_nav(scenario)
    # 从scenario中提取期望的终点坐标和性能阈值
    goal = (scenario.goal['x'], scenario.goal['y'])
    th = scenario.effective_thresholds()
    results = [
        assertions.assert_arrival(result.final_position, goal, scenario.arrival_tolerance_m),
        assertions.assert_within_time(result.elapsed_s, th['budget_s']),
    ]
    assert result.status == 'SUCCEEDED', f'nav status = {result.status}'
    # 检查所有测试结果，如果其中有任何一个失败，就抛出异常并打印详细的失败总结。
    assert all(r.passed for r in results), assertions.summarize(results)
    report.render_html(scenario, results)


@pytest.mark.parametrize('scenario', ['edge_close_target'], indirect=True)
def test_functional_close_goal(ros, sut, scenario):
    result = run_nav(scenario)
    goal = (scenario.goal['x'], scenario.goal['y'])
    # 只检查有没有到达（因为距离太近，不比时间）
    results = [assertions.assert_arrival(result.final_position, goal,
                                         scenario.arrival_tolerance_m)]
    assert result.status == 'SUCCEEDED'
    assert all(r.passed for r in results), assertions.summarize(results)