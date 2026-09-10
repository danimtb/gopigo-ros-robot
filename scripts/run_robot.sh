#!/usr/bin/env bash
# Start the GoPiGo3 convoy node on a Raspberry Pi after rsync.
# Usage:
#   ./scripts/run_robot.sh a
#   ./scripts/run_robot.sh b
# The first robot with no leader on the network becomes leader after leader_timeout.
set -euo pipefail

ID="${1:-a}"
HERE="$(cd "$(dirname "$0")" && pwd)"
if [[ -d "$HERE/build/Release" ]]; then
  ROOT="$HERE"
elif [[ -n "${GOPIGO_ROOT:-}" ]]; then
  ROOT="$GOPIGO_ROOT"
else
  ROOT="$HOME/gopigo3-ros"
fi

if [[ "$ID" != "a" && "$ID" != "b" ]]; then
  echo "robot id must be a or b" >&2
  exit 1
fi

cd "$ROOT"
if [[ -f relocate_env.sh ]]; then
  bash relocate_env.sh
fi
# shellcheck disable=SC1091
. build/Release/generators/conanrun.sh

#export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# Fast DDS is the stand default. If the Conan tree has rmw_zenoh_cpp:
#   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
#   ros2 run rmw_zenoh_cpp rmw_zenohd   # one robot or the laptop

exec ./build/Release/gopigo3_ros_node --ros-args \
  -r "__ns:=/gopigo_${ID}" \
  -p "robot_id:=${ID}" \
  -p "convoy_enable:=true" \
  -p "convoy_role:=follower" \
  -p "line_follow:=true"
