#!/usr/bin/env bash

# utitilities for shell scripts for launching ROS packages

killall roscore rosmaster gzclient gzserver &> /dev/null

# kill all subprocesses on termination
set -e
trap 'kill 0 &> /dev/null' SIGINT EXIT

# function to open an xterm window, execute a command, and redirect the stderr to the calling tty
xterm_exec() {
    cmd="$1 2> $(tty)"

    font="fixed"
    size=11
    geom=120x24

    xterm -geometry "${geom}" -fa "${font}" -fs "${size}" -e "${cmd}" &
}

# function to wait for a rosnode or rostopic to be brought up
wait_ros() {
    suffix="$1"
    query="$2"

    case "${suffix}" in
      node|topic)
        ;;
      *)
        echo "Error: First parameter can be either \"node\" or \"topic\"."
        return 1
        ;;
    esac


    while [[ -z "$("ros${suffix}" list 2> /dev/null | grep "${query}")" ]]; do
        sleep 0.5
    done
}
