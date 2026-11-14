# nav2-regression-testbed

基于 ROS2 Jazzy + Gazebo Harmonic + TurtleBot3 + pytest 的 Nav2 导航回归测试平台，覆盖功能正确性、鲁棒性与性能三个质量维度，支持 Mock SUT 与真实 Gazebo 仿真 SUT 双轨运行。

---

## 这是什么

我在学习 Nav2 导航的时候，发现移动机器人从 A 点安全到达 B 点的导航能力是测试中最核心的一环。然而传统做法依赖人工启动仿真、肉眼观察轨迹，这种方式存在以下主要局限：
- 回归不可复现：每次手动操作，场景和输入都较难保持完全一致；
- 鲁棒性难覆盖：传感器丢包、噪声、延迟等异常工况在正常演示中很少出现；
- 结果难量化：到达精度、话题频率、资源占用缺乏统一的断言标准。

于是就有了这个项目（Nav2 导航回归测试台），平台把导航测试工程化：用场景配置驱动测试执行，用故障注入模拟异常工况，用断言库统一判定标准，使导航回归可以在 CI 中重复运行。
平台只依赖被测系统（SUT）对外暴露的标准接口——/navigate_to_pose action、/odom、/scan、/tf，因此同一套测试脚本可以对接两种 SUT：
- Mock SUT：纯 Python 实现的虚拟 TurtleBot3（20Hz 控制环 + P 控制器 + action 服务端），不依赖 Gazebo，秒级启动，适合 CI 与开发阶段快速回归；
- 真实 SUT：Gazebo Harmonic + Nav2 + TurtleBot3 全栈仿真，用于验证测试框架在真实导航栈下的表现。

## 项目结构

```
nav2-regression-testbed/
├── package.xml                      # ROS2 包的“身份证”：名字、依赖
├── setup.py                         # 告诉 colcon：Python 包怎么装、有哪些可执行节点
├── setup.cfg                        # ament_python 的标准配置（一般不用改）
├── resource/
│   └── nav_test_harness             # 空文件，ROS 用它标记“这是个已安装的包”
├── src/
│   └── nav_test_harness/            # 真正的 Python 源码包
│       ├── __init__.py              # 标记为 Python 包
│       ├── scenario.py              # 【核心】把 YAML 场景读成结构化对象
│       ├── orchestrator.py          # 【核心】驱动 /navigate_to_pose action 完成一次导航
│       ├── fault_injection.py       # 【核心】故障注入节点（丢包/延迟/噪声/TF抖动）
│       ├── metrics.py               # 度量节点：话题频率 + CPU/内存
│       ├── replay.py                # rosbag2 记录/回放（数据驱动回归）
│       ├── assertions.py            # 断言：把测量结果变 PASS/FAIL
│       ├── report.py                # 生成 HTML 报告 + 轨迹图
│       └── mock_sut.py              # 【核心】假 TurtleBot3，不依赖 Gazebo 也能跑
├── launch/                          
│   ├── bringup_sut_mock.launch.py   # 拉起 mock SUT（学习/CI 用）
│   ├── bringup_sut_real.launch.py   # 真实 SUT 模板（Gazebo+Nav2，需你改路径）
│   └── test_navigation.launch.py    # 最小 launch，手动看节点用
├── tests/
│   ├── helpers.py                   # 测试工具：切故障参数、等 action 就绪
│   ├── conftest.py                  # pytest 夹具：启动/关停 SUT、加载场景
│   ├── test_nav_functional.py       # 功能正确性用例
│   ├── test_nav_robustness.py       # 鲁棒性（故障注入）用例
│   ├── test_nav_performance.py      # 性能用例
│   ├── test_replay_regression.py    # rosbag 回放回归用例
│   └── test_launch_testing_smoke.py # launch_testing 演示
├── scenarios/*.yaml                 # 测试场景（= 测试即配置）
├── config/
│   ├── perf_thresholds.yaml         # 性能阈值
│   └── nav2_params.yaml             # Nav2 参考参数（真实 SUT 用）
├── worlds/                          # 简单 SDF 世界
├── scripts/
│   ├── run_local.sh                 # 一键构建 + 跑测试
│   └── record_demo.sh               # 记录 bag + 出报告/轨迹图
├── .github/workflows/ci-headless.yaml  # GitHub Actions（无界面 CI）
└── README.md
```

## 数据流

```
你用 pytest 跑一个用例
        │
        ▼
conftest.py 拉起 SUT（默认 mock_sut + 4 个故障注入节点）
        │
        ▼
用例从 scenarios/*.yaml 读出“起点/目标/故障/阈值”
        │
        ▼
orchestrator 给 /navigate_to_pose 发目标 → mock 开始“导航”
        │
        ├── mock 内部：控制器 → /cmd_vel_raw → [注入器] → /cmd_vel → 运动(plant)
        │                     /scan_raw  → [注入器] → /scan  → 控制器(避障)
        │                     /odom_raw  → [注入器] → /odom → 测试观测
        │                     /tf_raw    → [tf_jitter]→ /tf
        ▼
测试收集 result + metrics → assertions 判 PASS/FAIL → report 存 HTML
```

## 怎么跑

### 验证环境

```
ros2 pkg prefix nav2_msgs        # 应输出路径，说明包在
colcon --help                    # 有输出即 OK
python3 -c "import yaml, matplotlib, psutil; print('py deps OK')"   # 输出 py deps OK
```

### 构建与验证

```
cd ~/ros2_ws/src/nav2-regression-testbed
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_test_harness
source install/setup.bash

# 验证是否成功
ros2 pkg prefix nav_test_harness      # 输出路径 = 装好了
ros2 pkg executables nav_test_harness # 应列出 mock_sut / topic_injector / tf_jitter / perf_monitor
```

### Mock 版（不需要 Gazebo，秒级出结果）

```
# 终端 A：起 SUT
ros2 launch nav_test_harness test_navigation.launch.py

# 终端 B：另开一个
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav_test_harness
source install/setup.bash
python3 -m pytest tests/test_nav_functional.py -v
```

### 真实版（Gazebo + Nav2）

```
SUT_LAUNCH=bringup_sut_real.launch.py python3 -m pytest tests/test_nav_functional.py -v
```

注意：当前版本 conftest.py 直接调 ros2 launch nav2_bringup tb3_simulation_launch.py，wrapper 完全绕开了。
真实 SUT 启动需要等 Gazebo 加载完，大概 60-90 秒。

### 生成报告 + 轨迹图 + bag

```
# 终端A
source install/setup.bash
ros2 launch nav_test_harness bringup_sut_mock.launch.py

# 终端B
./scripts/record_demo.sh simple_line
# 打开 results/reports/simple_line.html 看报告
# 打开 results/plots/simple_line.png 看轨迹图
```

## 故障注入怎么做到的

不改 Nav2 任何代码，在传感器话题和 Nav2 之间插一层 relay 节点：

```
LiDAR → /scan_raw → [injector] → /scan → Nav2
```

四种模式，通过 ROS2 参数动态切换，不用重启任何东西：

- **passthrough**：原样转发
- **drop**：按概率丢消息，模拟传感器偶尔抽风
- **delay**：扣住消息再发，模拟延迟
- **noise**：加高斯噪声，模拟传感器精度下降

## 场景示例

```
name: simple_line
start: {x: 0.0, y: 0.0, yaw: 0.0}
goal: {x: 2.0, y: 0.0, yaw: 0.0}
timeout_s: 40
arrival_tolerance_m: 0.2
faults:
  - {kind: drop, topic: /scan, rate: 0.3}
thresholds:
  budget_s: 30.0
```

加新测试就加一个 YAML 文件，不用改 Python 代码。

## 环境要求

- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic（真实版需要，Mock 版不需要）
- Python 3.12+

## License

MIT
