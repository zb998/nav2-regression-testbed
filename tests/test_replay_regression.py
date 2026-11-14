"""
数据驱动回归测试：把机器人的真实行动轨迹“录”下来（当黑匣子），然后再把这段录像“放”出来，看看正不正常

@author: zbb
@date: 2026/10/17
@version: v1.0
"""
import time
import pytest
from nav_test_harness import replay
from helpers import run_nav


@pytest.mark.skipif(not replay.is_available(), reason='ros2 CLI / rosbag2 not available')
@pytest.mark.parametrize('scenario', ['simple_line'], indirect=True)
def test_replay_record_and_play(ros, sut, scenario, tmp_path):
    bag_dir = str(tmp_path / 'nav_bag')
    rec = replay.record(bag_dir, topics=['/odom'])
    time.sleep(2.0)     # 给录像机一点启动时间
    try:
        result = run_nav(scenario)
    finally:
        replay.stop(rec)
    assert result.status == 'SUCCEEDED'
    bag_path = tmp_path / 'nav_bag'
    # 文件夹存在，且里面不是空的
    assert bag_path.exists() and any(bag_path.iterdir()), 'rosbag not created'
    play = replay.play(bag_dir, use_sim_time=False)
    try:
        time.sleep(3.0)
    finally:
        replay.stop(play)
    # 检查播放器的退出状态：0（正常退出）、None（还没死）、或者负数（被我们主动 kill 掉的），都算正常
    assert play.returncode in (0, None) or play.returncode < 0, \
        f'ros2 bag play exited with {play.returncode}'