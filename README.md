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
This is because a white border increases the contrast between the marker and its surroundings, making it easier for the camera to detect. This is especially important if it is placed in an environment with similar colors or if the lighting conditions are not ideal. The white border also helps to make the detection more robust.

Here you can see the difference between the detection of ARUCO markers with black or white boxes:

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/aruco_black.png" width="300"> <img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/aruco_white.png" width="300">

### Robot model ###

This is the robot model used in the assignment:

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/rosbot_model.png" width="200">

It is based on the [rosbot2](https://github.com/Marslanali/husarion-rosbot-2-gazebo-simulation/tree/master/rosbot_description) robot, which provides the commercial rosbot2 equipped with a laser scanner and an RGB-D camera. For the assignment's goal, the robot must detect all the markers *without* moving the base. For this reason, the starting model has been changed with:
 - yawl joint: a continuous joint controlled in velocity to rotate the camera.
 - vertical joint: a prismatic joint that allows the camera to move up and down, controlled in position.
 - pitch joint: a rotational joint that allows the pitch motion of the camera, controlled in position.
 
Every PID controller's joint has been tuned manually, and the values are stored in the [config/move_camera_config.yaml](config/move_camera_config.yaml) file.

### Assumptions ###

The whole scenario has the following assumptions:
 - The robot is equipped with sensors that provide it with accurate odometry information, such as wheel encoders.
 - The robot has a laser scanner.
 - The robot's environment can be represented by a 2D map.
 - The environment does not change in time.
 - The battery can become low at any time.
 - Even if the battery is low, the robot goes to the recharging location only if it is reachable; otherwise, it continues traveling into the environment.
 - If the battery goes down while building the map, the robot finishes the build and then recharges.
 - The starting and the recharging positions can be different.

## Software architecture ##

Given the scenario and the assumptions, the software architecture is developed accordingly.

### Component diagram ###

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/component_diagram.png" width="900">

The Component diagram shows the interfaces between each node of the architecture. The light blue nodes are the ones implemented in this package.
The `find_marker`, `planner`, and `controller` modules have the same interfaces with the `state_machine` node as in the first assignment.

All the nodes will be discussed in detail in their own section.
The custom messages and services `SetRoomPosition.srv`, `RoomInformation.srv`, `MoveCamera.msg`, and `AskPosition.srv` are described below in the dedicated section.


### Sequence diagram ###

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/sequence_diagram.png" width="900">

### States diagram ###

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/states_diagram.png" width="900">

### ROS messages and actions ###

For building interfaces between nodes, in this package there are some custom messages and actions:
 - `MoveCamera.msg`: raise, pitch and omega commands for changing the robot's field of view.
 - `RoomConnection.msg`: couple of strings that identifies a door and a location in the environment.
 - `AskPosition.srv`: rosbot state interface, to ask the robot's or location's position.
   - *Request*: the name of the location or the robot.
   - *Reply*: the x, y coordinates of the requested object in space.
 - `RoomInformation.srv`: it carries the meaning of a detected marker.
   - *Request*: marker id number.
   - *Reply*: name of the room, its x and y position and a list of RoomConnection messages.
 - `SetRoomPosition.srv`: tell the rosbot state whic is the position of a location.
   - *Request*: room's name, and its x and y coordinates.
   - *Reply*: bool set to true when the work is completed.

## Project Structure

### Package List

This repository contains a ROS package named `patrolling_robot` that includes the following resources:
 - [config/](config/): RViz and controllers configurations.
    - [move_camera_config.yaml](config/move_camera_config.yaml): configuration file for the joint state controller and for all the joint controllers added to the robot.
    - [sim.rviz](config/sim.rviz): configuration file for RViz.
 - [docs/](docs/): it contains the documentation source.
 - [launch/](launch/): it ontains the launchfiles to launch this package.
    - [gazebo_environment.launch](launch/launchgazebo_environment.launch): launcher for the GAZEBO environment with the robot and the world loaded.
    - [move_base.launch](launch/move_base.launch): launcher for the move_base node.
    - [patrolling_robot.launch](launch/patrolling_robot.launch): global launcher of the node.
    - [simulation_gmapping.launch](launch/simulation_gmapping.launch): launcher of the mapping algorithm.
 - [media/](media/): it contains all images, gifs, videos, and diagrams of this readme.
 - [meshes/](meshes/): it contains the meshes of the robot's urdf.
 - [msg/](msg/): it contains messages exchanged through ROS topics.
    - [MoveCamera.msg](msg/MoveCamera.msg): message to command the movements of the camera.
    - [RoomConnection.msg](msg/RoomConnection.msg): message to build the semantic of the environment.
 - [param/](param/): it stores the parameters of the architecture.
    - [base_local_planner_params.yaml](param/base_local_planner_params.yaml): parameters of the local planner.
    - [costmap_common_params.yaml](param/costmap_common_params.yaml): parameters for both the global and local costmap.
    - [global_costmap_params.yaml](param/global_costmap_params.yaml): parameters for the global costmap.
    - [local_costmap_params.yaml](param/local_costmap_params.yaml): parameters for the local costmap.
    - [move_base_params.yaml](param/move_base_params.yaml): parameters for the move_base node.
 - [scripts/](scripts/): it contains the implementation of the python software components.
    - [controller.py](scripts/controller.py): module to implement the controller node, to make the robot reach the goal.
    - [planner.py](scripts/planner.py): module to implement the planner node, to generate a list of waypoints.
    - [rosbot_state.py](scripts/rosbot_state.py): module to implement the rosbot_state node, to store the robot's and locations' positions.
 - [src/](src/): it contains the implementation of the c++ software components.
    - [find_markers.cpp](src/find_markers.cpp): module to implement the find_marker node, to search for markers around the robot.
    - [marker_server.cpp](src/marker_server.cpp): module to retrieve informations from markers' id.
    - [move_camera.cpp](src/move_camera.cpp): module to make the camera movements easier.
 - [srv/](srv/): it contains the custom ROS services.
    - [SetRoomPosition.srv](srv/SetRoomPosition.srv): service to tell the position of the room.
    - [RoomInformation.srv](srv/RoomInformation.srv): service to retrieve informations about a marker id.
    - [AskPosition.srv](srv/AskPosition.srv): service to ask the position of an element(location or robot).
 - [urdf/](urdf/): it conteins the robot model.
    - [macros.xacro](urdf/macros.xacro): xacro file for building the robot's model.
    - [materials.xacro](urdf/materials.xacro): xacro file for the materials of the robot's model.
    - [rosbot.gazebo](urdf/rosbot.gazebo): file for showing and controlling the robot on gazebo.
    - [rosbot.xacro](urdf/rosbot.xacro): xacro file for the urdf model of the rosbot2.
    - [rosbot_modified.xacro](urdf/rosbot.xacro): xacro file for the urdf of the rosbot with some links and joints to move the camera.
 - [worlds/](worlds/): it contains the GAZEBO world.
    - [assignment_world.world](worlds/assignment_world.world): original world of the assignment.
    - [assignment_world2.world](worlds/assignment_world2.world): world with white boxes behind ARUCO markers.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.

### Dependencies ###

The software dependencies are:


## Software Components

It follows the details of each software component implemented in this repository, which is available
in the `scripts/` and `srv/` folder.
