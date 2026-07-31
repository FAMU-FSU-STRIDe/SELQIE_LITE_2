#!/bin/bash

# Get the absolute path to the directory containing this script
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Get the ROS workspace
ROS2_WS=${SCRIPT_DIR}/../..

# Start a new tmux session
tmux new-session -d -s selqie

# Split the window into a 2x2 grid
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

# Keep SELQIE's ROS graph isolated by default. The Fast DDS
# ParticipantEntitiesInfo/Bad alloc errors are commonly caused by discovering
# incompatible ROS 2 participants on the same network/domain. Override these
# before running this script if external ROS machines must connect.
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-1}
SETUP_CMD="export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}; source ${ROS2_WS}/install/setup.bash"
PYTHON_CAN_CHECK='python3 -c "import can" || { echo "ERROR: Missing python-can. Install it with: sudo apt install python3-can"; exec bash; }'

# Launch SELQIE in top-left
tmux select-pane -t 0
tmux send-keys "${SETUP_CMD}; ${PYTHON_CAN_CHECK}; ros2 launch selqie_bringup selqie_hw.launch.py" C-m

# Sourced environment in bottom-left
tmux select-pane -t 1
tmux send-keys "${SETUP_CMD}" C-m

# SELQIE Terminal in top-right
tmux select-pane -t 2
tmux send-keys "${SETUP_CMD}; ros2 run selqie_ui selqie_terminal" C-m

# Jetson Stats in bottom-right
tmux select-pane -t 3
tmux send-keys "jtop" C-m

# Source tmux config file to enable bindings and mouse support
tmux source-file $SCRIPT_DIR/.tmux.conf

# Attach to the session
tmux attach-session -t selqie
