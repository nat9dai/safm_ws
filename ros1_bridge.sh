#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
# source ~/wsa/ros1_msgs_ws/devel/setup.bash # need this if you want to "bridge-all-topics"
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

trap 'kill 0' SIGINT SIGTERM

# check user's first argument
# Usage: ./ros1_bridge.sh 1 -> bridge all topics
#        ./ros1_bridge.sh 2 -> bridge specified topics (bridge_topics.yaml)
if [ -n "$1" ]; then
  mode="$1"
else
  echo "Bridge mode: [1] All topics  [2] Specified topics (bridge_topics.yaml)"
  read -rp "Select (1/2): " mode
fi

ros2 launch conversion_pkg conversion.launch.py > /dev/null 2>&1 &

if [ "$mode" = "1" ]; then
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
  # the topics frequencies drop a lot
else
  SCRIPT_DIR="$(dirname "$0")"
  rosparam load "$SCRIPT_DIR/bridge_topics.yaml"
  ros2 run ros1_bridge parameter_bridge
fi

wait
