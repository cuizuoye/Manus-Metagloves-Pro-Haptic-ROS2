#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 2 ]]; then echo "Usage: $0 /path/to/MANUS_Core_3.2.1_SDK_Linux.tar.gz WINDOWS_CORE_IPV4" >&2; exit 2; fi
archive=$(realpath "$1")
core_ip=$2
python3 - "$core_ip" <<'PY'
import ipaddress,sys
ipaddress.IPv4Address(sys.argv[1])
PY
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace="$repo_dir/ros2_ws/src"
for d in manus_ros2 manus_ros2_msgs ManusSDK; do
  if [[ -e "$workspace/$d" ]]; then echo "Already exists: $workspace/$d; refusing overwrite" >&2; exit 1; fi
done
stage=$(mktemp -d "$repo_dir/.sdk-stage.XXXXXXXX")
trap 'rm -rf "$stage"' EXIT
tar -xzf "$archive" -C "$stage" 'ROS2 package/manus_ros2' 'ROS2 package/manus_ros2_msgs' 'ROS2 package/ManusSDK'
for d in manus_ros2 manus_ros2_msgs ManusSDK; do mv "$stage/ROS2 package/$d" "$workspace/$d"; done
python3 - "$workspace" <<'PY'
from pathlib import Path
import sys
src=Path(sys.argv[1])/'manus_ros2/src'
for name in ('ManusDataPublisher.hpp','ManusDataPublisher.cpp'):
 p=src/name; p.write_bytes(p.read_bytes().replace(b'\r\n',b'\n'))
PY
patch -p1 -d "$workspace" < "$repo_dir/patches/manus-3.2.1-remote.patch"
python3 - "$workspace/manus_ros2/src/ManusDataPublisher.hpp" "$core_ip" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1]); text=p.read_text(); assert text.count('<WINDOWS_CORE_IP>')==1; p.write_text(text.replace('<WINDOWS_CORE_IP>',sys.argv[2]))
PY
echo "SDK prepared. Build ros2_ws with colcon after sourcing ROS 2."
