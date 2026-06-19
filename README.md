# Autonomous Patrol Robot System for detecting abnormal situations in buildings
Intelligent Robotics final project

### Team
Innu

### Name
김세은(Seun Kim)

## Project Introduction
 This project is an autonomous patrol robot system that autonomously moves along the patrol path inside the building and checks items that fit the characteristics of that point when they arrive at a specific patrol point.
The robot uses LiDAR in the main passage to check the presence of obstacles, and in the restricted area, it uses RGB camera images to detect the Gazebo factor of the human role. The check results are delivered to ROS 2 custom message, and normal and abnormal situations are recorded in log files and visualized through RViz and OpenCV screens.

## AI Use
Retrieve references and help to resolve errors

## Resources
- Intelligent Robotics - ROS2 Lecture Materials - Lecture Materials for Intelligent Robotics - Ewha Womans University:
https://github.com/Ewha-AIRLab/intelligent-robotics-ros2/tree/main
- ROS 2 Humble Documentation - Official documentation for ROS2: https://docs.ros.org/en/humble/index.html
- Navigation2 Tutorials - Example implementations for connecting custom mobile robots to the ROS2 Navigation Stack:
https://github.com/ros-navigation/navigation2_tutorials
- SLAM Toolbox - ROS2 package used for 2D SLAM, occupancy grid map generation, and map saving:
https://github.com/SteveMacenski/slam_toolbox
- ros_gz - ROS2 packages for connecting Gazebo Sim topics, sensors, and simulation entities with ROS2:
https://github.com/gazebosim/ros_gz
- vision_opencv - ROS2 packages including cv_bridge for converting ROS image messages into OpenCV images:
https://github.com/ros-perception/vision_opencv
- OpenCV - Computer vision library used for HSV color conversion, thresholding, morphology, contour extraction, and visualization:
https://github.com/opencv/opencv
- RViz Marker Display Types - Official ROS2 tutorial for visualizing points, arrows, boxes, cylinders, text, and other markers in RViz:
https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html

## Demo Video


## Environment
|Environment|
|---|
|Windows 11|
|WSL2 Ubuntu 22.04|
|ROS2 Humble|
|Gazebo Fortress|
|Python 3.10|

## Tools
ROS 2, Gazebo Sim, SLAM Toolbox, Nav2, RViz, OpenCV
