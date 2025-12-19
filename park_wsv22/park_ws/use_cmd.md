rosrun parking_demo real_map_parking_node
source devel/setup.bash
rosparam set /vehicle_wheelbase 2.7; rosparam set /vehicle_max_steering 0.6; rosrun parking_demo real_map_parking_node


rosparam set /vehicle_wheelbase 3 && rosparam set /vehicle_max_steering 0.6 && rosrun parking_demo real_map_parking_node

source /home/goldminer/park_ws/devel/setup.bash && roslaunch /home/goldminer/park_ws/src/parking_demo/launch/parking.launch
source devel/setup.bash && roslaunch parking_demo parking.launch
 source /home/goldminer/park_ws/devel/setup.bash && rosparam set /lookahead_distance 1 && rosparam set /lookahead_min 0.2 && rosparam set /lookahead_max 0 && rosparam set /curvature_speed_gain 4.0 && rosparam set /steering_rate_limit 1.0 && rosrun parking_demo real_map_parking_node
rosparam set /kp_speed 4.0 && rosparam set /curvature_speed_gain 1.0 && rosparam set /min_turning_radius 2.0 && rosparam set /max_steering_angle 1.0 && rosparam set /min_speed -2.0
sudo apt install ros-$ROS_DISTRO-ompl*

cd /home/goldminer/park_ws && source devel/setup.bash && rosparam set /lookahead_distance 0.5 && rosparam set /lookahead_min 0 && rosparam set /lookahead_max 0.2 && rosparam set /curvature_speed_gain 1.0 && rosparam set /steering_rate_limit 1.0 && rosrun parking_demo real_map_parking_node
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
roslaunch parking_demo parking.launch
