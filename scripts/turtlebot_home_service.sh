#!/usr/bin/env bash

# kill all subprocesses on termination
set +m
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

# function to open an xterm window, execute a command, and wait for a node to be brought up
xterm_run() {
    cmd="$1"
    node="$2"

    FONT="fixed"
    SIZE=11

    xterm -fa "fixed" -fs 11 -e "${cmd}" &
    while [[ -z "$(rosnode list 2> /dev/null | grep "${node}")" ]]; do
       sleep 0.5
    done
}

# launch Gazebo world
echo -n "Launching Gazebo... "
xterm_run "roslaunch turtlebot_gazebo turtlebot_world.launch" "/gazebo"
echo "DONE"

# launch RViz with configuration
echo -n "Launching RViz... "
xterm_run "rosrun rviz rviz -d $(rospack find add_markers)/../rviz_config/turtlebot_config.rviz" "/rviz"
echo "DONE"

# launch amcl and wait for it
echo -n "Launching AMCL... "
xterm_run "roslaunch turtlebot_gazebo amcl_demo.launch" "/amcl"
echo "DONE"

# start pick_objects node and wait for it
echo -n "Starting the pick_objects node... "
xterm_run "rosrun pick_objects pick_objects" "/pick_objects"
echo "DONE"

# start add_markers node and wait for it
echo -n "Starting the add_markers node... "
xterm_run "rosrun add_markers add_markers" "/add_markers"
echo "DONE"

echo "Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
