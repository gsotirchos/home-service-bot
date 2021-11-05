cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

add_compile_options(-std=c++11)

# required for cpp language server
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  visualization_msgs
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  message_generation
  roscpp
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  MarkerPose.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})

add_dependencies(pick_objects
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
