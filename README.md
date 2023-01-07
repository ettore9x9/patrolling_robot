# Patrolling robot #

**A ROS-based simulation of a patrolling robot in a close environment.**  
Author: *Ettore Sani* ettoresani0@gmail.com

---

## Introduction ##

This repository contains ROS-based software that simulates a patrolling robot.

You can find the documentation for this repository at this [link](https://ettore9x9.github.io/patrolling_robot/).

This software architecture has been developed for the second assignment of the Experimental Laboratory class at the University of Genoa.

In particular, the software developed is based on the first assignment of the course, which you can find on this [GitHub](https://github.com/ettore9x9/surveillance_robot.git).
The code in this repository is designed to simulate an environment with [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs), using a [URDF](http://wiki.ros.org/urdf) model for the robot. It utilizes [OpenCV](http://wiki.ros.org/vision_opencv) to detect ARUCO markers and the [move_base](http://wiki.ros.org/move_base) package to map the environment, plan, and control the robot's movements. The visualization tool [RViz](http://wiki.ros.org/rviz) is also used in this code.
The software is provided in Python 3 and c++ 11.

## Scenario ##

The scenario is the same as the first assignment, despite some improvements:
 1. Phase 1:
    - The robot starts in the E location of the environment.
    - The robot detects ARUCO markers without moving its base.
    - At each marker detected corresponds some information about the environment.
    - The robot builds the topological map of the environment.
 2. Phase 2:
    - The robot moves through locations following a surveillance policy.
    - For moving to a new location, the robot must plan a path and control its position.
    - After reaching a location, the robot scans the surroundings by rotating the camera.
    - When the battery is low, the robot goes to the E location and waits for recharging.
    - If a room has not been visited for some time, it becomes urgent.
    
### Environment ###

The indoor environment is composed of locations and doors:
 - E is the starting and recharging location.
 - A location with only one door is a room.
 - A location with more than one door is a corridor.
 - If two locations have the same door, the robot can move from one to the other.
 
 The Gazebo worlds are in the [/worlds](https://github.com/ettore9x9/patrolling_robot/tree/main/worlds) folder.
 The default one is *assignment_world2.world*. It is different from the given one because of ARUCO detection problems.
 
In particular, the modified world only changes the color of the box behind the ARUCO marker from black to white.
This because a white border is used to increase the contrast between the marker and its surroundings, making it easier for the camera to detect the marker. This is especially important if the marker is placed in an environment with similar colors to the marker, or if the lighting conditions are not ideal. The white border also helps to make the marker more robust.

Here you can see the difference between the detection of aruco markers with black or white boxes:

