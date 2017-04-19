# turtlebot_walker
This ROS package contains a node that implement a simple algorithm that makes the turtlebot behave as a roomba cleaning robot

## Overview
The turtlebot ros node is a node the subscribes to the laser scan topic in the ros network and uses the information received in order to guide the turtlebot around obstacles. The algorithm used is very simple and consists of a series of if statements that determines whether the robot is too close to object in front of it, or in some case objects on the right and left corners of the robot. If the distance between the robot and the objects detected is too small, then the robot will perform a 90 degree rotation in a direction away from the object detected. This node is tested in the gazebo simulation platform and in order to test the algorithm performance more obstacle can be added the basic gazebo_world simulation. 
## How to build and run turtlebot_walker node
The turtlebot_walker node is built by running the catkin_make command inside the catkin_ws workspace. This command will compile all nodes inside the workspace. After the node has been compiled, it can be run by typing:
roslaunch turtlebot_walker turtlebot_gazebo_walker.launch
The launch file will start the gazebo simulation engine and load the gazebo_world.It will also record a bag file containing selected broadcasted topics.
## Recording rosbag files with roslaunch files
In order to record a bag file using ros launch files you should do the following:
Add the rosbag as a node inside the launch file using the appropiate xml format.
Save the file.
Run the ros launch file as mentioned in the section above. The rosbag will start recording for as long as the simulation is running. In order to stop recording, you can kill the simulation with Ctrl+C and that should stop and save the bagfile. The bagfile gets store inside the home/.ros/ directory by default. You can either store it in the default directory or you could customized the rosbag xml section of the ros launch file in order to store it inside a results folder inside your ros package.

