#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/../.."
source tools/task2b/env.sh
mkdir -p log/task2b
pids=()
cleanup() { kill -TERM "${pids[@]}" 2>/dev/null || true; wait || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
fixture="$PWD/src/edgeai_rescue_coordinator/test/fixtures/task1_targets.complete.example.json"
install/edgeai_mission_interpreter/lib/edgeai_mission_interpreter/mission_interpreter --ros-args -p frozen_targets:="$fixture" >log/task2b/interpreter.log 2>&1 & pids+=($!)
install/edgeai_rescue_coordinator/lib/edgeai_rescue_coordinator/mission_validator --ros-args -p frozen_targets:="$fixture" >log/task2b/validator.log 2>&1 & pids+=($!)
for n in 1 2; do
  install/edgeai_system_tests/lib/edgeai_system_tests/mock_robot_agent --ros-args -r __ns:=/robot_$n -p robot_id:=robot_$n -p linear_speed:=2.0 -p angular_speed:=3.0 >log/task2b/robot_$n.log 2>&1 & pids+=($!)
done
python tools/task2b/mock_allocator.py --ros-args -p frozen_targets:="$fixture" >log/task2b/allocator.log 2>&1 & pids+=($!)
if [ "${1:-}" = --test ]; then
  python tools/task2b/acceptance.py
else
  echo 'TEST ONLY Mock stack running; Ctrl-C stops all nodes. Logs: log/task2b/'
  wait -n
fi
