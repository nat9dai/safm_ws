#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source ~/wsa/ros1_msgs_ws/devel/setup.bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
