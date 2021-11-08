#!/usr/bin/env bash

source "$(rospack find home_service_bot)/../scripts/utils.sh" || exit

# launch Gazebo world
echo -n "- Launching Gazebo... "
xterm_run "roslaunch turtlebot_gazebo turtlebot_world.launch"
wait_ros node "/gazebo"
echo "DONE"

# launch AMCL and wait for it
echo -n "- Launching AMCL... "
xterm_run "roslaunch turtlebot_gazebo amcl_demo.launch"
wait_ros node "/amcl"
echo "DONE"

# start RViz with configuration
echo -n "- Launching RViz... "
xterm_run "rosrun rviz rviz -d $(rospack find home_service_bot)/../rviz_config/turtlebot_config.rviz"
wait_ros node "/rviz"
echo "DONE"

# start pick_objects node and wait for it
echo -n "- Starting the pick_objects node... "
xterm_run "rosrun pick_objects pick_objects"
wait_ros node "/pick_objects"
echo "DONE"

# start add_markers node and wait for it
echo -n "- Starting the add_markers node... "
xterm_run "rosrun add_markers add_markers"
wait_ros node "/add_markers"
echo "DONE"

echo "- Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
