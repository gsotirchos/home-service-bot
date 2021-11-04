#!/usr/bin/env bash

source "$(rospack find home_service_bot)/../scripts/util.sh"

# launch Gazebo world
echo -n "- Launching Gazebo... "
xterm_exec "roslaunch mapping_bot world.launch"
wait_ros node "/gazebo"
echo "DONE"

# launch RTAB-Map localization
echo -n "- Launching RTAB-Map localization & move_base... "
xterm_exec "roslaunch mapping_bot localization.launch rviz:=false move_base:=true"
wait_ros node "/rtabmap"
wait_ros node "/move_base"
echo "DONE"

# start RViz with configuration
echo -n "- Launching RViz... "
xterm_exec "rosrun rviz rviz -d $(rospack find home_service_bot)/../rviz_config/config.rviz"
wait_ros node "/rviz"
echo "DONE"

# wait for map to load and robot localization
echo -n "- Waiting to load map and localize the robot... "
wait_ros topic "/move_base/global_costmap/footprint"
echo "DONE"

echo "- Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
