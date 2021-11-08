#!/usr/bin/env bash

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

# start the pick_objects node
echo -n "- Starting the pick_objects node... "
xterm_exec "rosrun pick_objects pick_objects"
wait_ros node "/pick_objects"
echo "DONE"

echo "- Robot moving for PICKUP at (x: 6.0, y: 4.0, rot: 1.0)"
rosservice call /pick_objects/move_robot \
"x: 6.0
y: 4.0
rot: 1.0"

# watch the /pick_objects/robot_state topic and wait for success
robot_state=2
while [[ "${robot_state}" != 0 ]]; do
    robot_state="$(rostopic echo -n 1 /pick_objects/robot_state 2> /dev/null | head -1 | tail -c 2)"

    if [[ "${robot_state}" == 1 ]]; then
        echo "- The robot has failed to reach the PICKUP location"
        echo "- Press Ctrl+C to close everything"
        read -r -d '' _ < /dev/tty
    fi
done
echo "- PICKUP location reached successfully"

echo "- Sleeping for 5 sec"
sleep 5

echo "- Robot moving for DROPOFF at (x: -5.0, y: -1.0, rot: 4.0)"
rosservice call /pick_objects/move_robot \
"x: -5.0
y: -1.0
rot: 4.0"

# watch the /pick_objects/robot_state topic and wait for success
robot_state=2
while [[ "${robot_state}" != 0 ]]; do
    robot_state="$(rostopic echo -n 1 /pick_objects/robot_state 2> /dev/null | head -1 | tail -c 2)"

    if [[ "${robot_state}" == 1 ]]; then
        echo "- The robot has failed to reach the PICKUP location"
        echo "- Press Ctrl+C to close everything"
        read -r -d '' _ < /dev/tty
    fi
done
echo "- DROPOFF location reached successfully"

echo "- Press Ctrl+C to close everything"
read -r -d '' _ < /dev/tty
