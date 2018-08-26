# Map My World Robot
## Abstract

This paper implements Simultaneous Localization and Mapping (SLAM) technique to construct a map of a given environment. A Real Time Appearance Based Mapping (RTAB-Map) approach was taken for accomplishing this task. Initially, a 2d occupancy grid  and 3d octomap was created from a provided simulated environment. Next, a personal simulated environment was created for mapping as well. In this appearance based method, a process called Loop Closure is used to determine whether a robot has seen a location before or not. In this paper, it is seen that RTAB-Map is optimized for large scale and long term SLAM by using multiple strategies to allow for loop closure to be done in real time and the results depict that it can be an excellent solution for SLAM to develop robots that can map an environment in both 2d and 3d.

## Introduction

In SLAM (Simultaneous Localization and Mapping), a robot must  construct a map of the environment, while simultaneously localizing itself relative to this map. This problem is more challenging than localization or mapping, since neither the map nor the robot poses are provided making this problem a 'chicken or a egg' problem. With noise in the robot's motion and measurements, the map and robot's pose will be uncertain, and the errors in the robot's pose estimates and map will be correlated. The accuracy of the map depends on the accuracy of the localization and vice versa. Given a series of sensor observations over discrete time steps , the SLAM problem is to compute an estimate of the agent’s location and a map of the environment.
In this paper, two simulation environments were provided where SLAM was performed. The robot was successfully able to localize itself and map the 3d world. The benchmark environment is called kitchen-dining (Figure 1) and the second environment is that of a cafeteria called sagar-cafe (Figure 2).


![Kitchen-dining world](images/udacity_world/watermarked/gazebo_udacity_world.jpeg "Figure 1. Robot model in Kitchen-Dining world")

Figure 1: Robot model in Kitchen-Dining world

![Sagar-Cafe world](images/sagar_cafe_world/gazebo_sagar_cafe_world.jpeg "Figure 2: Robot model in Cafe-world")

Figure 2: Robot model in Cafe-world

## Background

The FastSLAM algorithm uses a custom particle filter approach to solve the full SLAM problem with known correspondence. Using particles, FastSLAM estimates a posterior over the robot's path along with the map. Each of these particles hold the robot's trajectory which gives an advantage to SLAM to solve the problem of mapping with known poses. In addition to the trajectory, each particle holds a map and each feature of the map is represented by a local Gaussian.

With the FastSLAM algorithm, the problem is now divided into two separate independent problems, each of which aims to solve the problem of estimating features of the map. To solve these independent mini-problems, FastSLAM will use the low dimensional Extended Kalman Filter. While map features are treated independently, dependency only exists between robot pose uncertainty. This custom approach of representing the posterior with particle filter and Gaussian is known by Rao-Blackwellized particle filter approach. The Grid based FastSLAM is really an extension of FastSLAM and it adapts FastSLAM to grid maps.

With grid mapping algorithm, the environment can be modeled using grid maps without predefining any landmark position. So by extending the FastSLAM algorithm to occupancy grid maps, the SLAM problem can now be solved in an arbitrary environment. While mapping the real world environment, mobile robots equipped with range sensors can be used and the FastSLAM algorithm can be extended to solve the SLAM problem in terms of grid maps.

The first term in the RHS represents the robot trajectory where just as in FastSLAM, with the grid based FastSLAM, each particle holds a guess of the robot's trajectory.

The second term represents a map where each particle maintains its own. The grid based FastSLAM algorithm will update each particle by solving the mapping with known poses problem using the Occupancy grid mapping algorithm.

RTAB-Map (Real Time Appearance Based Mapping) is a graph based SLAM approach. Appearance based SLAM means that the algorithm uses data collected from vision sensors to localize the robot and map the environment. In appearance based methods, a process called Loop Closure is used to determine whether the robot has seen a location before. As the robot travels to new areas in its environment, the map is expanded and the number of images that each new image must be compared to increases. This causes the loop closure to take longer with the complexity increasing linearly. RTAB-Map is optimized for large scale and long term SLAM by using multiple strategies to allow for loop closure to be done in real time. Figure 3 shows the block diagram of the front end and the back end.

![RTAB-Map general graph](images/udacity_world/rtabmap_general_graph.png "Figure 3: RTAB-Map general graph")

Figure 3: RTAB-Map block diagram

