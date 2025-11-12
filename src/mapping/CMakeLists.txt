cmake_minimum_required(VERSION 2.8.3)
project(mapping)

# Find packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)

find_package(tf2_ros REQUIRED)

# Catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_msgs tf2 tf2_ros geometry_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Executables
add_executable(map_aggregator src/map_aggregator.cpp)

# Link libraries
target_link_libraries(map_aggregator 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# If you have other executables, add them the same way
# add_executable(other_node src/other_node.cpp)
# target_link_libraries(other_node ${catkin_LIBRARIES})