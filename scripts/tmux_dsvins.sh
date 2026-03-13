#!/bin/bash

set -e # Exit immediately if an error occurs

# TODO: Timesync

GCS_IP="192.168.1.100"
SESSION_NAME="dsvins_session"
ROS1_SOURCE="source /opt/ros/noetic/setup.bash"
WS_DIR="$HOME/wyc/"

# Kill old session
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session and split layout: LEFT, RIGHT-TOP , RIGHT-BOTTOM
readarray -t PANES < <(tmux list-panes -t "$SESSION_NAME" -F '#{pane_id}')
tmux new-session -d -s "$SESSION_NAME" -c "$WS_DIR"
tmux split-window -h -t "$SESSION_NAME" -c "$WS_DIR"

LEFT_PANE="${PANES[0]}"
RIGHT_PANE="${PANES[1]}"
BOT_LEFT_PANE=$(tmux split-window -v -t "$LEFT_PANE" -c "$WS_DIR" -P -F '#{pane_id}')
BOT_RIGHT_PANE=$(tmux split-window -v -t "$RIGHT_PANE" -c "$WS_DIR" -P -F '#{pane_id}')

# LEFT
tmux send-keys -t "$LEFT_PANE" "cd $WS_DIR && $ROS1_SOURCE" C-m
tmux send-keys -t "$LEFT_PANE" "roslaunch mavros px4.launch fcu_url:=\"/dev/ttyTHS0:3000000\" gcs_url:=\"udp://@${GCS_IP}:14550\"" C-m

# RIGHT-TOP for image driver
tmux send-keys -t "$RIGHT_PANE" "cd ${WS_DIR}oak_ffc_4p_ros" C-m
tmux send-keys -t "$RIGHT_PANE" "./start_docker.sh 1" C-m
tmux send-keys -t "$RIGHT_PANE" "source ./devel/setup.bash" C-m
tmux send-keys -t "$RIGHT_PANE" "roslaunch oak_ffc_4p_ros OV9782.launch" C-m

# BOTTOM LEFT for D2SLAM
tmux send-keys -t "$BOT_LEFT_PANE" "cd ${WS_DIR}D2SLAM" C-m
tmux send-keys -t "$BOT_LEFT_PANE" "./start_docker.sh 1" C-m
tmux send-keys -t "$BOT_LEFT_PANE" "source ./devel/setup.bash" C-m
tmux send-keys -t "$BOT_LEFT_PANE" "roslaunch d2vims quadcam.launch" C-m

# BOTTOM RIGHT for odom -> mavros vison pose
tmux send-keys -t "$BOT_RIGHT_PANE" "cd ${WS_DIR}adaptor_ws/" C-m
tmux send-keys -t "$BOT_RIGHT_PANE" "./devel/setup.bash" C-m
tmux send-keys -t "$BOT_RIGHT_PANE" "roslaunch vins_to_mavros vins_to_mavros.launch" C-m
