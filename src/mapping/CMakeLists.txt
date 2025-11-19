cmake_minimum_required(VERSION 3.0.2)
project(mapping)

## Compile as C++14
add_compile_options(-std=c++14 -O3 -march=native)

## Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    tf 
    tf2 
    tf2_ros
    tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## LPA* planner node
add_executable(planner_node src/lpa_star_planner.cpp)
target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)



