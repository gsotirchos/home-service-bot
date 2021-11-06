cmake_minimum_required(VERSION 3.0.2)
project(add_markers)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  tf2_geometry_msgs
  std_srvs
  roscpp
  pick_objects
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
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(add_markers src/add_markers.cpp)
target_link_libraries(add_markers ${catkin_LIBRARIES})

add_dependencies(add_markers
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
