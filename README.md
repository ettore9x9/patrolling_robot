# Patrolling robot #

**A ROS-based simulation of a patrolling robot in a close environment.**  
Author: *Ettore Sani* ettoresani0@gmail.com

---

## Introduction ##

This repository contains *ROS-based* software that simulates a patrolling robot.

You can find the documentation for this repository at this [link](https://ettore9x9.github.io/patrolling_robot/).

This software architecture has been developed for the second assignment of the *Experimental Laboratory* course at the *University of Genoa*.

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
This is because a white border increases the contrast between the marker and its surroundings, making it easier for the camera to detect. 
This is especially important if it is placed in an environment with similar colors or if the lighting conditions are not ideal. 
The white border also helps to make the detection more robust.

Here you can see the difference between the detection of ARUCO markers with black or white boxes:

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/aruco_black.png" width="300"> <img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/aruco_white.png" width="300">

As we can see, the same marker with the same position of the camera is detected only in the modified world.
This is because, in the first case, the MarkerDetector can not find the corners of the marker.

This is the overall world in GAZEBO, where the robot must move autonomously following the assignment's specifications:

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/gazebo_world.png" width="900">

In the world are present rooms and corridors, in particular, the map is the same as the default one of the first assignment.

### Robot model ###

This is the robot model used in the assignment:

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/rosbot_model.png" width="200">

It is based on the [rosbot2](https://github.com/Marslanali/husarion-rosbot-2-gazebo-simulation/tree/master/rosbot_description) robot, which provides the commercial rosbot2 equipped with a laser scanner and an RGB-D camera. For the assignment's goal, the robot must detect all the markers without moving the base. For this reason, the starting model has been changed with:
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
 - Markers' information matches the topology of the environment.

## Software architecture ##

Given the scenario and the assumptions, the software architecture is developed accordingly.

### Component diagram ###

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/component_diagram.png" width="900">

The Component diagram shows the interfaces between each node of the architecture. The light blue nodes are the ones implemented in this package.
The `find_marker`, `planner`, and `controller` modules have the same interfaces with the `state_machine` node as in the first assignment.

The `rosbot_state` component stores the positions of the locations and keeps track of the robot's position. At the first moment, it loads information through `SetRoomPosition.srv`. When needed by the `planner_node`, it shares the positions through the other service interface `AskPosition.srv`.

The `move_camera` node provides a service `MoveCamera.srv` that allows the client node to easily set commands to the joints that orient the camera.
It is used by the `find_markers` node to search markers and by the `controller` node to rotate the camera after reaching a location to patrol the area.


### Sequence diagram ###

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/sequence_diagram.png" width="900">

The Sequence diagram shows the flow of information between components. The main structure is inherited from the first assignment, with three more nodes: `find_markers` (instead of `find_qr`), `marker_server`, and `rosbot_state`.

As we can see, the `rosbot_state` collects information about rooms' positions by the `find_marker` node and shares them in a second moment with the `planner` node.
The `marker_server` node is used while building the semantics of the environment. The information provided is split by the `find_markers` node and stored in the ontology or in the `rosbot_state` node. The param `env/marker_number` stores the number of markers to detect; after their detection, the `find_markers` node exits. This is the transition between *phase 1* and *phase 2* above mentioned.

### States diagram ###

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/states_diagram.png" width="900">

The State diagram is very similar to the one of the first assignment; it changes only in the first phase, where there is the markers' detection.
In this case, the software architecture loops until all markers are found. In each loop, it finds a new marker, asks the `marker_server` node the semantic meaning of the detected marker, and stores it. When all markers are detected, the `state_machine` node calls the reasoner.

### ROS messages and actions ###

For building interfaces between nodes, in this package there are some custom messages and actions:
 - `MoveCamera.msg`: raise, pitch, and omega commands for changing the robot's field of view.
 - `RoomConnection.msg`: a couple of strings that identify a door and a location in the environment.
 - `AskPosition.srv`: `rosbot_state` interface, to ask the robot's or location's position.
   - *Request*: the name of the location or the robot.
   - *Reply*: the x and y coordinates of the requested object in space.
 - `RoomInformation.srv`: it carries the meaning of a detected marker.
   - *Request*: marker id number.
   - *Reply*: name of the room, its x and y position, and a list of RoomConnection messages.
 - `SetRoomPosition.srv`: tell the `rosbot_state` which is the position of a location.
   - *Request*: room's name, and its x and y coordinates.
   - *Reply*: bool set to true when the work is completed.

## Project Structure ##

### Package List ###

This repository contains a ROS package named `patrolling_robot` that includes the following resources:
 - [config/](config/): RViz and controllers configurations.
    - [move_camera_config.yaml](config/move_camera_config.yaml): configuration file for the *joint state controller* and for all the *joint controllers* added to the robot.
    - [sim.rviz](config/sim.rviz): configuration file for RViz.
 - [docs/](docs/): it contains the documentation source.
 - [launch/](launch/): it contains the launch files to launch this package.
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
    - [controller.py](scripts/controller.py): module to implement the `controller_node`, to make the robot reach the goal.
    - [planner.py](scripts/planner.py): module to implement the `planner` node, to generate a list of waypoints.
    - [rosbot_state.py](scripts/rosbot_state.py): module to implement the `rosbot_state` node, to store the robot's and locations' positions.
 - [src/](src/): it contains the implementation of the c++ software components.
    - [find_markers.cpp](src/find_markers.cpp): module to implement the `find_marker` node, to search for markers around the robot.
    - [marker_server.cpp](src/marker_server.cpp): module to implement the `marker_server` node to retrieve informations from markers' id.
    - [move_camera.cpp](src/move_camera.cpp): module to implement the `move_camera` node to make the camera movements easier.
 - [srv/](srv/): it contains the custom ROS services.
    - [SetRoomPosition.srv](srv/SetRoomPosition.srv): service to tell the position of the room.
    - [RoomInformation.srv](srv/RoomInformation.srv): service to retrieve informations about a marker id.
    - [AskPosition.srv](srv/AskPosition.srv): service to ask the position of an element(location or robot).
 - [urdf/](urdf/): it conteins the robot model.
    - [macros.xacro](urdf/macros.xacro): xacro file for building the robot's model.
    - [materials.xacro](urdf/materials.xacro): xacro file for the materials of the robot's model.
    - [rosbot.gazebo](urdf/rosbot.gazebo): file for showing and controlling the robot on GAZEBO.
    - [rosbot.xacro](urdf/rosbot.xacro): xacro file for the urdf model of the rosbot2.
    - [rosbot_modified.xacro](urdf/rosbot.xacro): xacro file for the urdf of the rosbot with some links and joints to move the camera.
 - [worlds/](worlds/): it contains the GAZEBO world.
    - [assignment_world.world](worlds/assignment_world.world): original world of the assignment.
    - [assignment_world2.world](worlds/assignment_world2.world): the world with white boxes behind ARUCO markers.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.

### Dependencies ###

The software dependencies are:
 - [xterm](https://xtermjs.org/docs/): to open multiple terminals with the launcher.
 - [rospy](http://wiki.ros.org/rospy): to define ROS nodes, services and related messages.
 - [roscpp](http://wiki.ros.org/roscpp): to define ROS nodes, services and related messages.
 - [roslaunch](http://wiki.ros.org/roslaunch): to launch multiple nodes.
 - [message_generation](http://wiki.ros.org/message_generation): to generate custom messages.
 - [actionlib](http://wiki.ros.org/actionlib/DetailedDescription): to define action servers.
 - [patrolling_robot package](https://github.com/ettore9x9/surveillance_robot.git): for the `state_machine` and the `battery_manager` nodes.
 - [move_base](http://wiki.ros.org/move_base): for mapping, planning and controlling the robot.
 - [OpenCV](http://wiki.ros.org/vision_opencv): for marker detection.
 - [Gazebo](http://wiki.ros.org/gazebo_ros_pkgs): as simulation environment.
 - [RViz](http://wiki.ros.org/rviz): to visualize sensor data.

## Software Components

It follows the details of each software component implemented in this repository, which is available in the `scripts/` and `srv/` folders.

### The `find_markers` Node ###

The `find_markers` Node is able to find Aruco markers around the robot. It subscribes to image and turn data, processes the data to detect markers, and publishes the results.
It publishes to the `move_camera` node to let the markers enter the field of view of the camera, more in detail, the method *Loop* is used to decide how the camera must move and the method *CamCommand* is used for sending the camera commands.
Depending on the variable *turn*, which identifies the number of rotations around the z-axis of the camera, this method set different camera orientations.
Hopefully, after one rotation looking downward and one rotation upward, all markers are detected. If not, the pitch angle is set randomically.

```c++
  void Loop(){
    switch (turn){
    case 0:
      CamCommand(0.8, 0.034, 0.2);
      break;
    case 1:
      CamCommand(0.8, 0.034, -0.5);
      break;
    default:
      CamCommand(0.8, 0.034, rand()/RAND_MAX - 0.5);
      break;
    }
  }
 ```
  
Moreover, it publishes asynchronously the statements to the state_machine node, because one marker detection can correspond to multiple statements.
This is the code for the asynchronous publisher:

```c++
  void PublishStatement(){
    if (!statPairVec.empty()){
      surveillance_robot::Statement statToSend;
      mutex.lock();
      statToSend.location = statPairVec[0].first;
      statToSend.door = statPairVec[0].second;
      statPairVec.erase(statPairVec.begin());
      mutex.unlock();
      statToSend.stamp = ros::Time::now();
      statPub.publish(statToSend);
    }
  }
  ```
Where the global variable `statPairVec` is a vector of pairs of status information. 
The first element is the location, and the second is the door.
This variable is shared and asynchronously accessed, so it must be protected by a mutex.
  
This node also calls the `set_room_position` server to store the x y positions of each room.

Anyway, the principal function of this code is the callback to the image topic `ImageCb`: it processes the image data to detect markers, and stores the detected id in the *markersDetected* vector. It also calls the `marker_server` and `set_room_position` services and displays the image with the detected markers on the OpenCV window.

Regarding marker detection, it uses the *OpenCV ROS Bridge* to transform the ROS image to an OpenCV image, then detects the markers with the *ARUCO MarkerDetector*.

This is an example of the `find_markers` behavior, showing also the `state_machine` node storing the statements on the ontology.

https://user-images.githubusercontent.com/91745595/211189014-0d2dd848-6b06-4b32-93df-99dd5f0008a5.mp4

### The `move_camera` Node ###

The `move_camera` Node is an adapter from the *camera/camera_command* topic to all the topics needed for moving each joint of the robot.
It also subscribes to the */joint_states* topic and triggers the */camera/camera_turn* whenever the camera completed a full rotation.

The *CamCb* function extracts the values for omega, raise, and pitch from the message and publishes them to the appropriate topics:

```c++
  void CamCb(const patrolling_robot::MoveCamera command) {
  	std_msgs::Float64 omega;
  	std_msgs::Float64 raise;
  	std_msgs::Float64 pitch;
  	
  	omega.data = command.omega;
  	raise.data = command.raise;
  	pitch.data = command.pitch;

    rotatePub.publish(omega);
    raisePub.publish(raise);
    pitchPub.publish(pitch);
  }
```

This is an example of the node used for rotating the camera after the robot reaches a new location.

![rotate_camera](https://user-images.githubusercontent.com/91745595/211189025-92b2eaf9-9b7f-484d-8316-6cca6c1f07ec.gif)

### The `rosbot_state` Node ###

The `rosbot_state` Node manages the positions of rooms and the robot in the environment. 
It provides services for adding new rooms and asking for the positions of rooms and the robot, and it listens for updates to the robot's pose and stores it.

This service callback answer with the x and y position of the requested room or of the robot:

```python
def FindPosition(self, request):
	response = AskPositionResponse()
	if request.what == "rosbot":
		response.x = self.robotPoseX
		response.y = self.robotPoseY
	else:
		response.x = self.dictionary[request.what][0]
		response.y = self.dictionary[request.what][1]
	return response
```

The location's x and y positions are stored in a dictionary.

### The `planner` Node ###

The `planner` Node implements a service that, provided with the goal's name, asks for the actual robot's position and for the goal location's position, and returns a list of two waypoints (start, goal).

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/terminal.png" width="500">

### The `controller` Node ###

The `controller` Node calls the move_base action service. It implements a service that, provided with a list of waypoints, asks the move_base planning, mapping, and controlling algorithm to bring the robot sequentially through all of them. 

When a waypoint is reached, the robot turns the camera to look around, then starts again to reach the next waypoint, if any.

It is based on the structure of the third assignment of the *Research Track 1* course, which you can find at this [GitHub](https://github.com/ettore9x9/driving_modalities.git).

For autonomously driving the robot, the program sends a goal to the action server */move_base*, receiving feedback and monitoring the status until the goal is reached or canceled. Thanks to the gmapping algorithm, the robot can create a map of the surrounding environment during its tours, as shown in this picture:

<img src="https://github.com/ettore9x9/patrolling_robot/blob/main/media/rviz_map.png" width="900">

The move_base node implements the action server to control the robot through the shortest path to reach the given position. 
You can find all the parameters' tuning in the folder [param/](param/).
This is an example of the robot reaching a goal in the environment.

https://user-images.githubusercontent.com/91745595/211189092-38e84907-6e68-49f3-9a2c-a42065d9848d.mp4

After reaching the goal, the robot performs a scan of the environment by rotating the camera.

## Launching the Software ##

This software has been based on ROS Noetic, and it has been developed with this Docker-based [environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already provides the required dependencies listed above. 

### Installation ###

Follow these steps to install the software.
 - Clone the repository of the [first assignment](https://github.com/ettore9x9/surveillance_robot.git) inside your ROS workspace.
 - Switch on branch assignment2 `git checkout assignment2`.
 - Follow the installation for the repository of the first assignment.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.

### Launchers ###

Use the following command to launch the software:
```bash
roslaunch patrolling_robot patrolling_robot.launch
```
If you want to suppress the output, in particular the warnings related to the move_base node, and show more clearly the INFO outputs, run:
```bash
roslaunch patrolling_robot patrolling_robot.launch 2>/dev/null
```
Or use the following command just to display the robot in the environment:
```bash
roslaunch patrolling_robot gazebo_environment.launch
```

### ROS Parameters ###

This software requires the following ROS parameters.
 - `env/marker_number`: It stores the total number of markers to detect.
 - `test/recharging_time`: It represents the time required for the robot to get fully charged.
 - `test/random_sense/active`: It is a boolean value that activates (i.e., `True`) or deactivates (`False`) the random-based stimulus' generation. If this parameter is `True` the parameter below is also required.  If it is `False` the parameter below is not used.
 - `test/random_sense/battery_time`: It indicates the time passed within the battery state becomes low. It should be a list of two float numbers, i.e., `[min_time, max_time]` in seconds and the time passed after the robot starts moving after a recharge will be a random value within such an interval.

## Improvements ##

These are some possible improvements to this repository:
 - Implements fault management.
 - Build a hierarchy between urgent locations, and make the robot moves to the more urgent.
 - Implements the possibility of switching between the two phases arbitrarily.
 - When the robot has a low battery, let the architecture react immediately.
 - Implements the possibility of adding another robot.
 - Implements the possibility of having different recharging locations.

---
