#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source ~/wsa/ros1_msgs_ws/devel/setup.bash # don't need; px4_msgs built on Noetic
source /opt/ros/foxy/setup.bash
source install/local_setup.bash

colcon build --packages-select ros1_bridge --cmake-force-configure
