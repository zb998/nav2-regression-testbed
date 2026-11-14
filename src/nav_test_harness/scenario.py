"""
把我们写好的 YAML 配置文件（测试场景），读取并转换成 Python 程序可以直接操作和执行的“对象”。

@author: zbb
@date: 2026/08/28
@version: v1.0
"""

from __future__ import annotations
# dataclass: 装饰器，自动生成 __init__、__repr__ 等方法，非常适合用来定义纯数据类
# field: 用来给 dataclass 的属性设置默认值（特别是当默认值是可变类型如 dict, list 时）
# asdict: 把 dataclass 对象递归地转换成普通字典
from dataclasses import dataclass, field, asdict
# 用于处理文件路径
from pathlib import Path
import yaml


# 默认阈值
DEFAULT_THRESHOLDS = {
    'budget_s': 60.0,              # 预算时间：60秒内必须到达
    'arrival_tolerance_m': 0.2,    # 到达容差：距离终点 0.2 米内算成功
    'min_scan_hz': 8.0,            # 激光雷达最小频率：8Hz
    'max_cpu_pct': 80.0,           # 最大 CPU 占用率：80%
    'max_mem_pct': 70.0,           # 最大内存占用率：70%
}


@dataclass
class FaultSpec:
    """一条故障注入说明。kind: drop/delay/noise/tf_jitter"""
    kind: str               # 故障类型，必填（如 'drop' 丢包, 'delay' 延迟, 'noise' 噪声）
    topic: str = ''         # 故障作用于哪个 ROS 话题（默认空）
    rate: float = 0.0       # 丢包率（针对 drop 类型）
    delay_s: float = 0.0    # 延迟秒数（针对 delay 类型）
    noise_std: float = 0.0  # 噪声标准差（针对 noise 类型）
    enabled: bool = True    # 是否启用该故障（默认启用）

    def as_params(self) -> dict:
        """把故障描述翻译成注入器要设置的 ROS 参数。"""
        if self.kind == 'drop':
            return {'mode': 'drop', 'rate': self.rate}
        if self.kind == 'delay':
            return {'mode': 'delay', 'delay_s': self.delay_s}
        if self.kind == 'noise':
            return {'mode': 'noise', 'noise_std': self.noise_std}
        return {'mode': 'passthrough'}      # 如果是未知类型，默认直接放行


@dataclass
class Scenario:
    name: str
    description: str = ''
    sut: str = 'mock'       # 被测系统(System Under Test)：默认用假的 'mock'，也可以用真实的 'real'
    # 这里用了 field(default_factory=...) 是因为字典是可变对象，不能直接写 start={'x':0...}，必须用工厂函数每次生成新字典
    start: dict = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    # 终点坐标，默认去 (2,0)
    goal: dict = field(default_factory=lambda: {'x': 2.0, 'y': 0.0, 'yaw': 0.0})
    timeout_s: float = DEFAULT_THRESHOLDS['budget_s']
    arrival_tolerance_m: float = DEFAULT_THRESHOLDS['arrival_tolerance_m']
    # 该场景包含的故障列表，默认为空列表
    faults: list = field(default_factory=list)
    # 性能阈值字典（覆盖默认阈值），默认为空字典
    thresholds: dict = field(default_factory=dict)

    def effective_thresholds(self) -> dict:
        """计算最终生效的阈值（合并默认阈值和自定义阈值）"""
        merged = dict(DEFAULT_THRESHOLDS)
        # 用场景自定义的阈值去更新
        merged.update(self.thresholds)
        return merged

    def to_dict(self) -> dict:
        """把整个 Scenario 对象转换成字典，方便后续保存或序列化打印"""
        return {
            'name': self.name, 
            'description': self.description, 
            'sut': self.sut,
            'start': self.start, 
            'goal': self.goal,
            'timeout_s': self.timeout_s,
            'arrival_tolerance_m': self.arrival_tolerance_m,
            'faults': [asdict(f) for f in self.faults],
            'thresholds': self.thresholds
        }


def load_scenario(path):
    """从单个 YAML 文件路径加载场景"""
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    # 使用 **f 把字典里的键值对展开，实例化成一个个 FaultSpec 对象
    faults = [FaultSpec(**f) for f in raw.get('faults', [])]
    return Scenario(
        name=raw['name'],
        description=raw.get('description', ''),
        sut=raw.get('sut', 'mock'),
        start=raw.get('start', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
        goal=raw.get('goal', {'x': 2.0, 'y': 0.0, 'yaw': 0.0}),
        timeout_s=raw.get('timeout_s', DEFAULT_THRESHOLDS['budget_s']),
        arrival_tolerance_m=raw.get('arrival_tolerance_m',
                                    DEFAULT_THRESHOLDS['arrival_tolerance_m']),
        faults=faults,
        thresholds=raw.get('thresholds', {}),
    )


def load_scenario_dir(scenarios_dir):
    """从文件夹中批量加载所有 YAML 场景"""
    d = Path(scenarios_dir)
    # d.glob('*.yaml') 查找所有 yaml 文件
    # sorted() 按文件名排序，保证每次加载的顺序是一致的（这对测试复现很重要）
    return [load_scenario(p) for p in sorted(d.glob('*.yaml'))]