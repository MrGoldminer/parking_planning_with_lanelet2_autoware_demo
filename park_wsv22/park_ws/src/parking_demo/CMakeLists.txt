# CMakeLists.txtcmake_minimum_required(VERSION 3.0.2)
project(parking_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf
  roslib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Optional OMPL (for Reeds-Shepp planning). Will be used if found on the system.
# Manually set OMPL paths for ROS Noetic installation
set(OMPL_INCLUDE_DIRS /opt/ros/noetic/include/ompl-1.6)
set(OMPL_LIBRARIES /opt/ros/noetic/lib/x86_64-linux-gnu/libompl.so)

# Check if OMPL library file exists
if(EXISTS ${OMPL_LIBRARIES})
  message(STATUS "Found OMPL: enabling Reeds-Shepp planner support")
  include_directories(${OMPL_INCLUDE_DIRS} /usr/include/eigen3)
  add_definitions(-DHAVE_OMPL)
  set(OMPL_FOUND TRUE)
else()
  message(STATUS "OMPL not found, using centerline interpolation fallback")
  set(OMPL_FOUND FALSE)
endif()

add_executable(real_map_parking_node src/real_map_parking_node.cpp)
if(OMPL_FOUND)
  target_link_libraries(real_map_parking_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
else()
  target_link_libraries(real_map_parking_node ${catkin_LIBRARIES})
endif()