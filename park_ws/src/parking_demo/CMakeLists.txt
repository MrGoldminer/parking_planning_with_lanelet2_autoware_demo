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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES parking_demo_lib
  CATKIN_DEPENDS roscpp visualization_msgs geometry_msgs nav_msgs std_msgs tf roslib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

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

# ========== 模块化库 ==========
# 创建parking_demo_lib库（包含所有模块）
add_library(parking_demo_lib
  src/osm_map_loader.cpp
  src/graph_builder.cpp
  src/path_planner.cpp
  src/pure_pursuit_controller.cpp
  src/parking_maneuver.cpp
  src/visualizer.cpp
)

target_link_libraries(parking_demo_lib
  ${catkin_LIBRARIES}
)

# ========== 可执行文件 ==========

# 1. 原始版本（单体架构）
add_executable(real_map_parking_node src/real_map_parking_node.cpp)
if(OMPL_FOUND)
  target_link_libraries(real_map_parking_node ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
else()
  target_link_libraries(real_map_parking_node ${catkin_LIBRARIES})
endif()

# 2. 重构版本（模块化架构）
add_executable(parking_system_refactored src/parking_system_node_refactored.cpp)
target_link_libraries(parking_system_refactored
  parking_demo_lib
  ${catkin_LIBRARIES}
)

# ========== 安装规则 ==========
install(TARGETS
  parking_demo_lib
  real_map_parking_node
  parking_system_refactored
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)