#!/usr/bin/env bash

# utitilities and settings for shell scripts testing ROS packages

# kill all subprocesses on termination
set +m
trap 'kill $(jobs -p) &> /dev/null' SIGINT SIGTERM EXIT

killall roscore rosmaster gzclient gzserver &> /dev/null

# function to open an xterm window, execute a command, and wait for a ROS node to be brought up
xterm_exec() {
    cmd="$1"

    font="fixed"
    size=11
    geom=120x24

    xterm -geometry "${geom}" -fa "${font}" -fs "${size}" -e "${cmd}" &
}

wait_ros() {
    suffix="$1"
    query="$2"

    case "${suffix}" in
      node|topic)
        ;;
    *)
        echo "Error: First parameter can be either \"node\" or \"topic\"."
        return
        ;;
    esac


    while [[ -z "$("ros${suffix}" list 2> /dev/null | grep "${query}")" ]]; do
        sleep 0.5
    done
}
