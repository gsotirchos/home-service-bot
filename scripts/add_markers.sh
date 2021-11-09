#!/usr/bin/env bash

main() {
    source "$(rospack find home_service_bot)/../scripts/utils.sh" || exit

    # launch Gazebo world
    echo -n "- Launching Gazebo... "
    xterm_exec "roslaunch mapping_bot world.launch"
    wait_ros node "/gazebo"
    echo "DONE"

    # launch RTAB-Map localization & move_base
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

    # start the add_markers node
    echo -n "- Starting the add_markers node... "
    xterm_exec "rosrun add_markers add_markers"
    wait_ros node "/add_markers"
    echo "DONE"

    echo "- Adding PICKUP marker at (x: 6.0, y: 4.0, rot: 1.0)"
    rosservice call /add_markers/show_marker \
    "x: 6.0
    y: 4.0
    rot: 1.0"

    echo "- Sleeping for 5 sec"
    sleep 5

    echo "- Hiding PICKUP marker"
    rosservice call /add_markers/hide_marker

    echo "- Sleeping for 5 sec"
    sleep 5

    echo "- Adding DROPOFF marker at (x: -5.0, y: -1.0, rot: 5.0)"
    rosservice call /add_markers/show_marker \
    "x: -5.0
    y: -1.0
    rot: 4.0"

    echo "- Press Ctrl+C to close everything"
    read -r -d '' _ < /dev/tty
}

export -f main
bash -c main
unset -f main
