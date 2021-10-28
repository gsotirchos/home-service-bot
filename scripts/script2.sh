TURTLEBOT_GAZEBO_WORLD_FILE="${PWD}/maps/test.world"
roslaunch turtlebot_gazebo turtlebot_world.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch turtlebot_gazebo gmapping_demo.launch
roslaunch turtlebot_gazebo amcl_demo.launch
