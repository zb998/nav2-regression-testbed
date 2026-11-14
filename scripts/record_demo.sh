#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# 1. 修复 set -u 导致的 source 报错
set +u
source install/setup.bash
set -u

SCEN="${1:-simple_line}"
# 2. 每次运行前，强制清理旧的 bag，防止 rosbag 报错退出
rm -rf "results/bags/$SCEN"
mkdir -p results/bags results/reports results/plots

echo "Recording /odom + /scan to results/bags/$SCEN ..."
ros2 bag record -o "results/bags/$SCEN" --topics /odom /scan &
REC_PID=$!

echo "Running scenario: $SCEN"
python3 - "$SCEN" <<'PY'
import sys
from pathlib import Path
import rclpy
sys.path.insert(0, 'src')
from nav_test_harness.scenario import load_scenario
from nav_test_harness.orchestrator import NavigationTaskRunner
from nav_test_harness import assertions, report

rclpy.init()
try:
    sc = load_scenario(Path('scenarios') / f'{sys.argv[1]}.yaml')
    runner = NavigationTaskRunner(sc)
    result = runner.execute()
    runner.destroy_node()
finally:
    if rclpy.ok():
        rclpy.shutdown()

goal = (sc.goal['x'], sc.goal['y'])
results = [
    assertions.assert_arrival(result.final_position, goal, sc.arrival_tolerance_m),
    assertions.assert_within_time(result.elapsed_s, sc.timeout_s),
]
print(assertions.summarize(results))
report.render_html(sc, results, out_path=f'results/reports/{sc.name}.html')
report.plot_trajectory(result.trajectory, out_path=f'results/plots/{sc.name}.png')
report.write_results_json(sc, results, out_path=f'results/reports/{sc.name}.json')
print(f"report: results/reports/{sc.name}.html")
print(f"trajectory: results/plots/{sc.name}.png")
PY

kill "$REC_PID" 2>/dev/null || true
echo "bag: results/bags/$SCEN"