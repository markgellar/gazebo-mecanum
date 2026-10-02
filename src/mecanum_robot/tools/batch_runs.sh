#!/bin/bash
# Usage: tools/batch_runs.sh 101 102 103   (one headless exploration per seed)
# Run headless explorations with different noise seeds, one after another.
# Each stops at "Exploration complete" (or a 20 min timeout); leftovers are cleaned between runs.
S=$HOME/mecanum_ws/slam_records/batch_logs; mkdir -p $S
source /opt/ros/humble/setup.bash
source /home/markwilliam/mecanum_ws/install/setup.bash

cleanup() {
  pkill -INT -f "ros2 launch mecanum_robot" 2>/dev/null
  sleep 8
  pkill -9 -f "ros2 launch mecanum_robot" 2>/dev/null
  pkill -9 -f gzserver 2>/dev/null
  pkill -9 -f "mecanum_robot/lib/mecanum_robot" 2>/dev/null
  pkill -9 -f "robot_localization/ekf_node" 2>/dev/null
  pkill -9 -f robot_state_publisher 2>/dev/null
  sleep 3
}

for seed in "$@"; do
  cleanup
  log=$S/run_seed_$seed.log
  echo "$(date +%H:%M:%S) seed $seed: starting" | tee -a $S/batch.log
  setsid ros2 launch mecanum_robot gazebo.launch.py gui:=false seed:=$seed > $log 2>&1 &
  start=$(date +%s)
  while true; do
    sleep 10
    if grep -q "Exploration complete" $log; then
      echo "$(date +%H:%M:%S) seed $seed: exploration complete after $(( $(date +%s) - start ))s" | tee -a $S/batch.log
      sleep 5
      break
    fi
    if (( $(date +%s) - start > 1200 )); then
      echo "$(date +%H:%M:%S) seed $seed: TIMEOUT" | tee -a $S/batch.log
      break
    fi
  done
  cleanup
  grep -h "Run summary" $log | tail -1 | tee -a $S/batch.log
done
echo "$(date +%H:%M:%S) all done" | tee -a $S/batch.log
