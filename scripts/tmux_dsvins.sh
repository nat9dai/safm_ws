#!/bin/bash

set -e # Exit immediately if an error occurs

# TODO: Timesync

GCS_IP="192.168.1.100"
SESSION_NAME="dsvins_session"
ROS1_SOURCE="source /opt/ros/noetic/setup.bash"
WS_DIR="$HOME/wyc/"

# Kill old session
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session
tmux new-session -d -s "$SESSION_NAME" -c "$WS_DIR"

# Split layout
tmux split-window -h -t "$SESSION_NAME" -c "$WS_DIR"

readarray -t PANES < <(tmux list-panes -t "$SESSION_NAME" -F '#{pane_id}')

LEFT_PANE="${PANES[0]}"
RIGHT_PANE="${PANES[1]}"

BOT_LEFT_PANE=$(tmux split-window -v -t "$LEFT_PANE" -c "$WS_DIR" -P -F '#{pane_id}')
BOT_RIGHT_PANE=$(tmux split-window -v -t "$RIGHT_PANE" -c "$WS_DIR" -P -F '#{pane_id}')

tmux send-keys -t "$LEFT_PANE" "cd $WS_DIR && $ROS1_SOURCE" C-m
tmux send-keys -t "$LEFT_PANE" "rostopic hz /mavros/imu/data_raw" C-m

# RIGHT-TOP - Camera
tmux send-keys -t "$RIGHT_PANE" "cd ${WS_DIR}oak_ffc_4p_ros" C-m
tmux send-keys -t "$RIGHT_PANE" "sleep 2 && ./start_docker.sh 1" C-m
tmux send-keys -t "$RIGHT_PANE" "source ./devel/setup.bash" C-m
tmux send-keys -t "$RIGHT_PANE" "roslaunch oak_ffc_4p_ros OV9782.launch" C-m

# LEFT-BOTTOM - D2SLAM
tmux send-keys -t "$BOT_LEFT_PANE" "cd ${WS_DIR}D2SLAM" C-m
tmux send-keys -t "$BOT_LEFT_PANE" "sleep 4 && ./start_docker.sh 1" C-m
tmux send-keys -t "$BOT_LEFT_PANE" "source ./devel/setup.bash" C-m
tmux send-keys -t "$BOT_LEFT_PANE" "roslaunch d2vins quadcam.launch" C-m

# RIGHT-BOTTOM - VINS -> MAVROS
tmux send-keys -t "$BOT_RIGHT_PANE" "cd ${WS_DIR}" C-m
tmux send-keys -t "$BOT_RIGHT_PANE" "source /opt/ros/noetic/setup.bash" C-m
tmux send-keys -t "$BOT_RIGHT_PANE" "rostopic echo /d2vins/odometry " C-m

# Adjust layout
tmux select-layout -t "$SESSION_NAME" tiled

# Attach to session so it shows in current terminal
tmux attach -t "$SESSION_NAME"

