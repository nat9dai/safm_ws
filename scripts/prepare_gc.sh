#!/bin/bash

set -e # Exit immediately if an error occurs

ROS1_SOURCE="/opt/ros/noetic/setup.bash"
VRPN_DIR="wyc/vrpn_client_ros/"

# ============================================================================
# Function: Start Screen Session
# ============================================================================
# Usage: start_screen_session <session_name> <command>
# Creates a detached screen session with ROS2 environment sourced
start_screen_session() {
  local session_name=$1
  local command=$2

  echo "Starting screen session: $session_name"

  # Check if screen session already exists
  if screen -list | grep -q "$session_name"; then
    echo "  WARNING: Screen session '$session_name' already exists. Killing it..."
    screen -S "$session_name" -X quit
    sleep 1
  fi

  # Create new detached screen session with proper environment setup
  screen -dmS "$session_name" bash -c "$command; exec bash"
  echo "  ✓ Screen session '$session_name' started"
  sleep 1
}

# ============================================================================
# Module 1: VRPN ROS Bridge
# ============================================================================
VRPM_COMMAND="source devel/setup.bash && roslaunch vrpn_client_ros sample.launch"
start_screen_session "vrpn_ros_bridge" "$VRPM_COMMAND"

# ============================================================================
# Startup Complete
# ============================================================================
echo ""
echo "======================================"
echo "All modules started successfully!"
echo "======================================"
echo ""
echo "Active screen sessions:"
screen -list
echo ""
echo "To attach to a screen session, use:"
echo "  screen -r <session_name>"
echo ""
echo "Available sessions:"
echo "  - vrpn_ros_bridge: VRPN ROS Bridge"
echo ""
echo "To detach from a screen session, press: Ctrl+A, then D"
echo "To stop all sessions, run: ./stop_system.sh"
echo "======================================"
