"""
ROS初始化，SUT调出，场景加载

@author: zbb
@date: 2026/11/08
@version: v1.3
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy

# __file__ 是当前 conftest.py 的路径，.parent 是 tests/ 目录，再 .parent 是项目根目录
REPO = Path(__file__).resolve().parent.parent
# 把 src/ 目录加到 Python 环境变量里，这样测试代码才能 import 到 nav_test_harness
sys.path.insert(0, str(REPO / 'src'))
# 把 tests/ 目录也加进去，为了能直接 import helpers.py
sys.path.insert(0, str(Path(__file__).resolve().parent))

from helpers import wait_for_action_server, set_initial_pose  # noqa: E402

SCENARIO_DIR = REPO / 'scenarios'


@pytest.fixture(scope='session')
def ros():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope='session')
def sut(request, ros):
    """
    【核心】每个测试用例跑之前，拉起一个 SUT (被测系统)。
    默认拉起 Mock 版本，跑完测试后负责把系统彻底杀死。
    """
    launch_file = os.environ.get('SUT_LAUNCH', 'bringup_sut_mock.launch.py')
    is_real = 'real' in launch_file

    if is_real:
        env = os.environ.copy()
        extra = '/opt/ros/jazzy/share/turtlebot3_gazebo/models:' \
                '/opt/ros/jazzy/share/nav2_minimal_tb3_sim/models'
        env['GZ_SIM_RESOURCE_PATH'] = env.get('GZ_SIM_RESOURCE_PATH', '') + ':' + extra
        cmd = ['ros2', 'launch', 'nav2_bringup', 'tb3_simulation_launch.py']
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=None,
                                preexec_fn=os.setsid, env=env)
    else:
        cmd = ['ros2', 'launch', 'nav_test_harness', launch_file]
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                preexec_fn=os.setsid)

    try:
        # action server 1.3秒就advertise了，但Gazebo还没加载完
        # 所以先等5秒确认subprocess活着，再等action server
        time.sleep(5)
        if proc.poll() is not None:
            raise RuntimeError(f'launch subprocess died immediately, rc={proc.returncode}')

        wait_for_action_server()

        # 关键：action server存在不代表Gazebo准备好了，再等5秒
        if is_real:
            time.sleep(5)
            set_initial_pose(1.6, 0.0, 0.0)
            time.sleep(5)

        yield proc
    finally:
        try:
            # 发送 SIGTERM 信号（相当于优雅地按 Ctrl+C），杀掉整个进程组
            # 为什么用 killpg？因为 ros2 launch 会派生几十个子节点，必须连根拔起
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                # 如果 10 秒后还死撑，直接发送 SIGKILL 强制拔电源
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def scenario_path(name):
    """辅助函数：根据文件名返回完整的 YAML 文件路径"""
    return SCENARIO_DIR / f'{name}.yaml'


@pytest.fixture()
def scenario(request):
    """
    【加载scenario】配合 @pytest.mark.parametrize 使用。
    把测试用例请求的scenario名字，转换成 Python 的 Scenario 对象。
    """
    from nav_test_harness.scenario import load_scenario
    # request.param 拿到的是装饰器里传进来的名字，比如 'simple_line'
    return load_scenario(scenario_path(request.param))