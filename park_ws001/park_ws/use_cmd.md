rosrun parking_demo real_map_parking_node
source devel/setup.bash
rosparam set /vehicle_wheelbase 2.7; rosparam set /vehicle_max_steering 0.6; rosrun parking_demo real_map_parking_node


rosparam set /vehicle_wheelbase 3 && rosparam set /vehicle_max_steering 0.6 && rosrun parking_demo real_map_parking_node


sudo apt install ros-$ROS_DISTRO-ompl*


catkin_make -DCMAKE_BUILD_TYPE=Release -j4
roslaunch parking_demo parking.launch
