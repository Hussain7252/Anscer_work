# Trajectory Saver and Visualizer
The project develops a trajectory visualization system in order to enable the user to be informed regarding the path followed by the robot and to be able to save and visualize it for future analysis.

## Overview
The project contains two nodes "trajectory_saver_node" and "trajectory_reader_node". The functionality of the nodes are as follows: -  

Trajectory_saver_node contains the service that allows the user to save the trajectory for the specified duration in a .json, .csv, .yaml file. Additionally, it also provides the operator the capability to visualize trajectory on the run.  

Trajectory_reader_node provides user the capability to read the path followed by the robot from the file and visualize it later for analysis.

## Algorithm
```pseudocode
1. Subscribe to the Odometry topic
2. For every Odometry message:
   - Extract (x, y) coordinates
   - Capture current timestamp
   - Append to an internal trajectory list
3. Wait for service call with desired format, filename, and duration
4. Filter trajectory points from the last <duration> seconds
5. Serialize and save to file in specified format (csv/json/yaml)
6. On a timer, publish MarkerArray to RViz:
   - Each recorded point visualized as a sphere
   - Published in the "odom" frame
```
## Parameters
 The following can be set in the `traj.yaml` file in the param folder.  
 For the trajectory_saver_node:
```
topic: "/pub_trajectory_markers" # Topic to publish marker array on while robot is on the run.
```
For the trajectory_reader_node:
```
file_path: "/home/hussain/my_path.json" # Path of the file from where trajectory data has to be read.
format: "json" # Format of the file in which trajectory data is stored.
topic: "/read_trajectory_markers" # Topic to publish trajectory marker arrays on.
```
## Build Instructions
Go to the base folder and run
```
colcon build
```
## Service
Make sure to launch the trajectory.launch.py before calling the server.
- Name: save_trajectory
- Type: trajectory::srv::SaveTrajectory
- Params: filename, format, duration
```
ros2 service call /save_trajectory trajectory/srv/SaveTrajectory "{filename: 'my_path.json', format: 'json', duration: 300.0}"
```
Call the service to save the trajectory.
## Launch Instructions
Launch the turtlebot environment: -
```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run turtlebot3_teleop teleop_keyboard
```
In order to launch the trajectory saver node launch the trajectory.launch.py file.
```
ros2 launch trajectory trajectory.launch.py
```
If plan to read the trajectory file and visualize it launch the node trajectory_reader.launch.py.
```
ros2 launch trajectory trajectory_reader.launch.py
```
## Rviz Setup
 - Set the Fixed Frame to `odom`

Add a display:

 - Click Add > choose MarkerArray
 - Set the topic 
