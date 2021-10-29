#!/usr/bin/env bash

# all subprocesses in same group id
set +m

# kill all subprocesses on termination
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

#TURTLEBOT_GAZEBO_WORLD_FILE="$(rospack find add_markers)/maps/test.world"

# launch gazebo world and wait for it
xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &
echo -n "Launching Gazebo... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/gazebo")" ]]; do
   sleep 0.5
done
echo "DONE"

# launch RViz with configuration and wait for it
xterm  -e  "rosrun rviz rviz -d $(rospack find add_markers)/../rviz_config/turtlebot_config.rviz" &
echo -n "Launching RViz... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/rviz")" ]]; do
    sleep 0.5
done
echo "DONE"

# launch amcl and wait for it
xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &
echo -n "Launching AMCL... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/amcl")" ]]; do
    sleep 0.5
done
echo "DONE"

# start add_markers node and wait for it
xterm  -e  "rosrun add_markers add_markers" &
echo -n "Starting the add_markers node... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/add_markers")" ]]; do
    sleep 0.5
done
echo "DONE"

echo "Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
