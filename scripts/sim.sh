#!/usr/bin/env bash
set -eo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ros_ws=${MANUS_ROS_WS:-"$repo_dir/ros2_ws"}
source /opt/ros/jazzy/setup.bash
source "$ros_ws/install/setup.bash"
set -u
python_bin=${MANUS_PYTHON:-"$repo_dir/.venv/bin/python"}
mode=${1:-offline}
if (($#)); then shift; fi
case "$mode" in
  offline) exec "$python_bin" "$repo_dir/sim/offline_demo.py" "$@" ;;
  live) exec "$python_bin" "$repo_dir/sim/shadow_teleop.py" "$@" ;;
  imu) exec "$python_bin" "$repo_dir/sim/imu_teleop.py" "$@" ;;
  test) exec "$python_bin" "$repo_dir/sim/shadow_teleop.py" --self-test "$@" ;;
  *) echo "Usage: $0 {offline|live|imu|test}" >&2; exit 2 ;;
esac
