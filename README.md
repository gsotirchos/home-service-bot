# home-service-bot

## Overview

This is a meta-package containing the packages for performing an object pickup and dropoff "home service" via the `add_markers` and `pick_objects` packages' nodes. [mapping_bot](https://github.com/7555G/mapping-bot)'s `rtabmap` (along with `teleop_twist_keyboard` and `move_base`) can be used for mapping the environment and then localizing the robot using the generated graph during the "home service".

* **add_markers**: Contains the `add_markers` node providing the services for manually showing and hiding a marker in RViz. It is also capable of notifying the `pick_objects` node to move the robot for pickup after a node is shown, and of hiding and re-showing the marker after successful pickup and dropoff.

* **pick_objects**: Contains the `pick_objects` node providing a service for conveniently sending navigation goals to `move_base`. It is also capable of notifying the `add_markers` node of the transferring and successful pickup and dropoff of the marker.

* [**mapping_bot**](https://github.com/7555G/mapping-bot): Contains the Gazebo environment and robot model along with the required configuration and launch files for bringing up the `rtabmap`, `rtabmapviz`, `move_base`, and `teleop_twist_keyboard` nodes.

### License

The source code is released under an [MIT license](LICENSE).

**Author/Maintainer: George Sotirchos**

The home-service-bot package has been tested under [ROS](https://www.ros.org) Kinetic in a docker container on Ubuntu 20.04 (see [Running in Docker](#running-in-docker) section). This is experimental, personal project code, and possibly subject to frequent changes with any need for explanation disclaimed.

<p align="center">
  <img src="./media/pickup.gif" width="47%">
  <img src="./media/dropoff.gif" width="47%">
</p>

## Installation

### Building from Source

#### Dependencies

- [ros-kinetic](http://wiki.ros.org)
- [ros-kinetic-rtabmap-ros](http://wiki.ros.org/rtabmap_ros)
- [ros-kinetic-move-base](http://wiki.ros.org/move_base)
- [ros-kinetic-teleop-twist-keyboard](http://wiki.ros.org/teleop_twist_keyboard)

#### Building

To build from source, with ROS Kinetic on Ubuntu 16.04, clone the latest version from this repository into your catkin workspace and compile the package with the following commands:

``` bash
mkdir -p /catkin_ws/src
cd catkin_ws/src
git clone --recurse-submodules https://github.com/7555G/home-service-bot
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
# place the downloaded database file at ./src/home-service-bot/mapping_bot/databases/rtabmap.db
```

The `rtabmap.db` database file (~500MB) containing a graph for localization can be downloaded from the following link:<br/>
https://drive.google.com/file/d/1b5pvZWY5gWn9cGNwS4kvx6GS7iLyPiKr/view?usp=sharing

### Running in Docker

Install [Docker](https://docs.docker.com/get-docker/).

Spin up a container with GUI forwarding for X11 applications:

``` bash
docker run \
    -ti \
    --rm \
    --network=host \
    --env="DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    --device=/dev/dri:/dev/dri \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --name ros-container \
    mjenz/ros-kinetic-desktop-full \
    bash
```

This downloads the `mjenz/ros-kinetic-desktop-full` image from [mjenz](https://hub.docker.com/u/mjenz)'s Docker repository, indicates that it requires an interactive terminal (`-t`, `-i`), gives it a name (`--name`), removes it after you exit the container (`--rm`), sets the required environment variables (`--env`) and access to local resources (`--device`, `--volume`) to be able to launch graphical applications (Gazebo, RViz, rqt_graph, etc.), and runs a command (`bash`).

Now, continue with the instructions from the [Building](#building) section.

## Usage

### Using shell scripts

The various features in this meta-package can be conveniently launched with the shell scripts in the `scripts` directory. The following scripts are available:

* [**scripts/test_slam.sh**](scripts/test_slam.sh): Starts the Gazebo simulation environment, and brings up the `rtabmap` node in *SLAM* mode along with an `rtabmapviz` window, and the `teleop_twist_keyboard` node.
* [**scripts/test_navigation.sh**](scripts/test_navigation.sh): Starts the Gazebo simulation environment, brings up the `rtabmap` node in *localization* mode along with the `move_base` node, starts a configured `rviz` window, and waits for the database map to load and the robot to be localized.
* [**scripts/pick_objects.sh**](scripts/pick_objects.sh): Starts the Gazebo simulation environment, brings up the `rtabmap` node in *localization* mode along with the `move_base` node, starts a configured `rviz` window, waits for the database map to load and the robot to be localized, brings up the `pick_objects` node, and moves the robot to the pickup and dropoff locations.
* [**scripts/add_markers.sh**](scripts/add_markers.sh): Starts the Gazebo simulation environment, brings up the `rtabmap` node in *localization* mode along with the `move_base` node, starts a configured `rviz` window, waits for the database map to load and the robot to be localized, brings up the `add_markers` node, and shows and hides a marker at the pickup and dropoff locations in 5 second intervals.
* [**scripts/home_service.sh**](scripts/home_service.sh): Starts the Gazebo simulation environment, brings up the `rtabmap` node in *localization* mode along with the `move_base` node, starts a configured `rviz` window, waits for the database map to load and the robot to be localized, brings up the `pick_objects` and `add_markers` nodes, and performs a pickup and dropoff: a pickup marker is shown, the robot moves to it, waits 5 sec, and then moves to another location where it places the marker.

### Using launch files

1. Start the Gazebo environment containing the robot:

    ``` bash
    roslaunch mapping_bot world.launch
    ```

2. Start the `rtabmap` node in *localization* mode along with the `move_base` node:

    ``` bash
    roslaunch mapping_bot localization.launch rviz:=false move_base:=true
    ```

   Alternatively, start the `rtabmap` node in *SLAM* mode along with `rtabmapviz`, and then the `teleop_twist_keyboard` node:

    ``` bash
    roslaunch mapping_bot mapping.launch
    roslaunch mapping_bot teleop.launch
    ```

3. Start the `pick_objects` node:

    ``` bash
    rosrun pick_objects pick_objects
    ```

4. Start the `add_markers` node:

    ``` bash
    rosrun add_markers add_markers
    ```

## Launch files

All the launch files used can be found in the [mapping_bot/launch](https://github.com/7555G/mapping-bot/tree/main/mapping_bot/launch) folder of the [mapping-bot](https://github.com/7555G/mapping-bot) package.

## Nodes

### add_markers

In addition to providing services for showing markers in RViz, this node implements a communication interface for cooperating with the `pick_objects` node by sharing state information using topics.

#### Published Topics

* `visualization_marker` ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

    The marker message for showing and hiding markers in RViz.

* `/add_markers/marker_state` ([std_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html))

    The current state of the marker. Can be one of the following:
    * Finished = 0<br/>
        No pickup or dropoff is to be executed. The robot's state has no effect on the marker state.
    * Pickup = 1<br/>
        A pickup is to be executed. If notified about the robot's state changing to Finished (0) then any markers will be hidden and the marker's state will advance to Dropoff (2).
    * Dropoff = 2<br/>
        A dropoff is to be executed. If notified about the robot's state changing to Finished (0) then a marker will be shown at the last goal issued to `move_base` and the marker's state will advance to Finished (0).

#### Subscribed Topics

* `/pick_objects/robot_state` ([std_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html))

    Receive information about changes to the robot's state. Specifically, when the robot's state changes to Finished (0) then either a pickup or a dropoff has been completed and a marker will have to be hidden or shown respectivelly.

* `/move_base/goal` ([move_base_msgs/MoveBaseActionGoal](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseActionGoal.html))

    Receive the last issued navigation goal for `move_base`. The goal's pose information is used to determine the location of the marker shown after a sucessful dropoff.

#### Services

* `~show_marker` ([pick_objects/MarkerPose](pick_objects/srv/MarkerPose.srv))

    Shows a marker at the requested location specified by the `x` coordinate, `y` coordinate, and `rot` rotation.

    ``` bash
    rosservice call /add_markers/show_marker \
    "x: 0.0
    y: 0.0
    rot: 0.0"
    ```

* `~hide_marker` ([std_srvs/Trigger](http://docs.ros.org/en/api/std_srvs/html/srv/Trigger.html))

    Hides any visible markers.

    ``` bash
    rosservice call /add_markers/hide_marker
    ```

### pick_objects

A [SimpleActionClient](https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionClient.html)\<[MoveBaseAction](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseAction.html)\> client is provided, which is capable of sending navigation goals to `move_base` and receiving the action's status information upon completion. This node also implements a communication interface for cooperating with the `add_markers` node by sharing state information using topics.

#### Published Topics

* `/pick_objects/robot_state` ([std_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html))

    The current state of the marker. Can be one of the following:
    * Finished = 0<br/>
        The robot has succesfully moved to the goal location. By sharing this state information the `add_markers` node is able to proceed to the next pickup/dropoff procedure state.
    * Failed = 1<br/>
        The robot has failed to move to the goal location. This state change does not advance the pickup/dropoff procedure.
    * Moving = 2<br/>
        The robot is now moving towards the goal location. This state change does not advance the pickup/dropoff procedure.

* `/move_base/goal` ([move_base_msgs/MoveBaseActionGoal](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseActionGoal.html))

    The navigation goal for `move_base` published via the `pick_objects` action client.

#### Subscribed Topics

* `/add_markers/marker_state` ([std_msgs/Int8](http://docs.ros.org/en/api/std_msgs/html/msg/Int8.html))

    Receive information about changes to the marker's state. Specifically, in the event of the marker's state changing to Pickup (1) while the robot is not moving (either Finished or Failed) then the robot will start moving towards the shown pickup marker.

* `visualization_marker` ([visualization_msgs/Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

    The marker message published by the `add_markers` node. This information is used to determine the goal location after a pickup marker is shown.

#### Services

* `~move_robot` ([pick_objects/MarkerPose](pick_objects/srv/MarkerPose.srv))

    Moves the robot to the requested location specified by the `x` coordinate, `y` coordinate, and `rot` rotation.

    ``` bash
    rosservice call /pick_objects/move_robot \
    "x: 0.0
    y: 0.0
    rot: 0.0"
    ```

### rtabmap

The aspects of the `rtabmap` node used in this project are the same as [those used in the maping-bot project](https://github.com/7555G/mapping-bot#rtabmap).

### move_base

The [SimpleActionServer](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionServer.html) server implementation provided by `move_base` is utilized to move the robot while monitoring its state by sending goals through the `pick_objects` node's [SimpleActionClient](https://docs.ros.org/en/diamondback/api/actionlib/html/classactionlib_1_1SimpleActionClient.html)\<[MoveBaseAction](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/msg/MoveBaseAction.html)\> client.

#### Action Subscribed Topics

* `/move_base/goal` ([move_base_msgs/MoveBaseActionGoal](http://docs.ros.org/en/api/move_base_msgs/html/msg/MoveBaseActionGoal.html))

    A goal for move_base to pursue in the world.

#### Action Published Topics

* `move_base/status` ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatusArray.html))

    Provides status information on the goals that are sent to the `move_base` action.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/7555G/home-service-bot/issues).

ROS: http://www.ros.org<br/>
RTAB-Map's ROS package: https://github.com/introlab/rtabmap_ros<br/>
move_base ROS package (ROS Navigation stack): https://github.com/ros-planning/navigation<br/>
teleop_twist_keyboard ROS package: https://github.com/ros-teleop/teleop_twist_keyboard
