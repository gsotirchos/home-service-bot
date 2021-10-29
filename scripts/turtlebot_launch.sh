#!/bin/sh

#TURTLEBOT_GAZEBO_WORLD_FILE="$(rospack find add_markers)/maps/test.world"

# launch world and wait for roscore
(xterm  -e  "roslaunch turtlebot_gazebo turtlebot_world.launch" &)

echo -n "Launching world... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/gazebo")" ]]; do
    sleep 0.5
done
echo "DONE"

# launch RViz with configuration and wait for it
(xterm  -e  "rosrun rviz rviz -d $(rospack find add_markers)/../rviz_config/turtlebot_config.rviz" &)

echo -n "Launching RViz... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/rviz")" ]]; do
    sleep 0.5
done
echo "DONE"

# launch amcl
(xterm  -e  "roslaunch turtlebot_gazebo amcl_demo.launch" &)

echo -n "Launching AMCL... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/amcl")" ]]; do
    sleep 0.5
done
echo "DONE"

# start add_markers node
(xterm  -e  "rosrun add_markers add_markers" &)

echo -n "Starting the add_markers node... "
while [[ -z "$(rosnode list 2> /dev/null | grep "/add_markers")" ]]; do
    sleep 0.5
done
echo "DONE"
