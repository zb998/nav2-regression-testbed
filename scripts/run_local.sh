#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "[1/3] Install python deps (best-effort)"
pip install --user pyyaml matplotlib psutil 2>/dev/null || true

echo "[2/3] Build the ament_python package"
colcon build --packages-select nav_test_harness
source install/setup.bash

echo "[3/3] Run the test suite against the mock SUT"
python3 -m pytest tests -v --tb=short "$@"

echo "Done. Reports land in ./results/"