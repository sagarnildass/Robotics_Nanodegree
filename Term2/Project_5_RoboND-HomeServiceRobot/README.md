# Home Service Robot Project

For this setup, catkin_ws is the name of the active ROS workspace. If your workspace name is different, change the commands accordingly.

If you do not have an active ROS workspace, you can create one by:

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Clone the required repositories to the `~/catkin_ws/src` folder. Note that this repository already includes official ROS packages compatible with this repository: [gmapping](https://github.com/ros-perception/slam_gmapping), [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop), [turtlebot_rviz_launchers](https://github.com/turtlebot/turtlebot_interactions), and [turtlebot_gazebo](https://github.com/turtlebot/turtlebot_simulator). Their dependencies must be installed to succesfully use this repository.

```sh
cd ~/catkin_ws/src
git clone https://github.com/kevinfructuoso/home-service-bot.git
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
cd ~/catkin_ws
catkin_make
```

To run the home service robot script file, open a terminal and run the following commands.
```sh
cd ~/catkin_ws
source devel/setup.bash
./src/home_service.sh
```

If everything is ready to rock'n'roll, the below is the result of running these commands (at 3x speed).

<p align="center"><img src="./resources/home-service-bot.gif"></p>