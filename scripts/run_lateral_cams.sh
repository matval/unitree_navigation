#!/bin/bash
SESSION_NAME="cameras_session"
SCRIPT="source catkin_ws/src/unitree_cameras/kill_processes.sh | tmux new-session -d -s '$SESSION_NAME' 'source catkin_ws/src/unitree_cameras/set_ros.sh && roslaunch unitree_cam_publisher run_cameras.launch'"

echo "Connecting to the belly Jetson Nano..."

SESSION_EXIST=$(sshpass -p "123" ssh -o StrictHostKeyChecking=no unitree@192.168.123.14 "tmux list-sessions | grep -o '$SESSION_NAME'")

echo "SESSION_EXIST: $SESSION_EXIST"

if [ -n "$SESSION_EXIST" ]; then
    # Session already exists, so kill it on the remote host
    sshpass -p "123" ssh -o StrictHostKeyChecking=no unitree@192.168.123.14 "tmux kill-session -t '$SESSION_NAME'"
fi

sshpass -p "123" ssh -o StrictHostKeyChecking=no unitree@192.168.123.14 "${SCRIPT}"

echo "Running lateral cameras!"