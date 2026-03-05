#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source ~/wsa/ros1_msgs_ws/devel/setup.bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

trap 'kill 0' SIGINT SIGTERM

echo "Bridge mode: [1] All topics  [2] Specified topics (bridge_topics.txt)"
read -rp "Select (1/2): " mode

ros2 launch conversion_pkg conversion.launch.py > /dev/null 2>&1 &

if [ "$mode" = "1" ]; then
  ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
else
  ros2 run ros1_bridge parameter_bridge \
    $(grep -v '^$' "$(dirname "$0")/bridge_topics.txt")
fi

wait
