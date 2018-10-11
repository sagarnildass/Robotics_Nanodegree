#!/bin/sh
catkin_dir=/home/sagarnil/catkin_ws
catkin_src_dir=$catkin_dir/src

#Launch turtlebot in the custom world
xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$catkin_src_dir/RoboND-HomeServiceRobotproject/Worlds/sagar_world.world" &
sleep 5

#Launch amcl demo
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$catkin_src_dir/RoboND-HomeServiceRobotproject/Worlds/myMap.yaml" &
sleep 2

#Launch rviz
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 10

#Launch markers in rviz
xterm -e " source $catkin_dir/devel/setup.bash; rosrun add_markers add_markers_node" &
sleep 3

#Launch navigation pick_objects node
xterm -e " source $catkin_dir/devel/setup.bash; rosrun pick_objects pick_objects_node"
