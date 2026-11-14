"""
封装 ros2 bag record/play 命令行，为数据驱动回归提供记录与回放

@author: zbb
@date: 2026/09/27
@version: v1.0
"""
from __future__ import annotations
import shutil
import subprocess
from pathlib import Path
from typing import Optional, Sequence


def is_available():
    # shutil.which 会在系统的环境变量 PATH 里找，找到返回路径，找不到返回 None
    return shutil.which('ros2') is not None


def record(bag_dir, topics=None, duration=None):
    """
    录制 rosbag。
    :param bag_dir: 录制的包保存路径（比如 'results/bag/my_record'）
    :param topics: 要录的话题列表（比如 ['/scan', '/odom']），不传则录全部
    :param duration: 录制时长（秒），不传则一直录直到手动停止
    """
    cmd = ['ros2', 'bag', 'record', '-o', str(bag_dir)]
    if topics:
        cmd += ['--topics', *topics]
    if duration is not None:
        cmd += ['--duration', str(duration)]
    # 启动子进程执行命令。
    # 【核心】使用 Popen 而不是 run，因为 Popen 是非阻塞的（异步的）。
    # 启动录制后，代码立刻往下走，不会卡在这里等录制结束。
    return subprocess.Popen(cmd)


def play(bag_dir, rate=1.0, use_sim_time=True, start_paused=False):
    """
    播放 rosbag。
    :param bag_dir: 要播放的包路径
    :param rate: 播放倍速（比如 2.0 表示 2 倍速快放）
    :param use_sim_time: 是否使用包里的时间（发布 /clock 话题）
    :param start_paused: 是否启动后立刻暂停（等外部信号再开始播）
    """
    cmd = ['ros2', 'bag', 'play', str(bag_dir), '--rate', str(rate)]
    if use_sim_time:
        cmd.append('--clock')
    if start_paused:
        cmd.append('--start-paused')
    return subprocess.Popen(cmd)


def stop(proc):
    """
    停止录制或播放的进程。
    :param proc: record() 或 play() 返回的子进程对象
    """
    if proc is not None:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()