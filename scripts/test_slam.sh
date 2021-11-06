#!/usr/bin/env bash

source "$(rospack find home_service_bot)/../scripts/utils.sh"

# launch Gazebo world
echo -n "- Launching Gazebo... "
xterm_exec "roslaunch mapping_bot world.launch"
wait_ros node "/gazebo"
echo "DONE"

# launch RTAB-Map SLAM
echo -n "- Launching RTAB-Map SLAM... "
xterm_exec "roslaunch mapping_bot mapping.launch"
wait_ros node "/rtabmap"
echo "DONE"

# launch teleop_twist_keyboard node
echo -n "- Starting the teleop_twist_keyboard node... "
xterm_exec "roslaunch mapping_bot teleop.launch"
wait_ros node "/teleop"
echo "DONE"

echo "- Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
