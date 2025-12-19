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
find_package(OMPL QUIET)
if(OMPL_FOUND)
  message(STATUS "Found OMPL: enabling Reeds-Shepp planner support")
  include_directories(${OMPL_INCLUDE_DIRS})
  add_definitions(-DHAVE_OMPL)
endif()

add_executable(real_map_parking_node src/real_map_parking_node.cpp)
if(OMPL_FOUND)
  target_link_libraries(real_map_parking_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
else()
  target_link_libraries(real_map_parking_node ${catkin_LIBRARIES})
endif()
