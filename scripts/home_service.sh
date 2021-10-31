#!/usr/bin/env bash

# kill all subprocesses on termination
set +m
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

killall roscore rosmaster

# function to open an xterm window, execute a command, and wait for a node to be brought up
xterm_exec() {
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
xterm_exec "roslaunch mapping_bot world.launch" "/gazebo"
echo "DONE"

# launch RTAB-Map localization
echo -n "Launching RTAB-Map localization... "
xterm_exec "roslaunch mapping_bot localization.launch rviz:=false move_base:=true" "/rtabmap"
echo "DONE"

# launch RViz with configuration
echo -n "Launching RViz... "
xterm_exec "rosrun rviz rviz -d $(rospack find home_service_bot)/../rviz_config/config.rviz" "/rviz"
echo "DONE"

# start the pick_objects node
echo -n "Starting the pick_objects node... "
xterm_exec "rosrun pick_objects pick_objects" "/pick_objects"
echo "DONE"

# start the add_markers node
echo -n "Starting the add_markers node... "
xterm_exec "rosrun add_markers add_markers" "/add_markers"
echo "DONE"

echo "Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
