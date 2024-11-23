# Fleet Robotics
## Fall 2024: Intro to Computational Robotics Final Project
### Contributors: Ivy, Vivian, Ariel, Charlie

# Abstract
This project aims to develop and implement a control system for a fleet of mobile robots, leveraging sensor fusion techniques for autonomous navigation and collision avoidance. The system combines internal odometry data from Neato robots (using ROS 2) with visual odometry derived from embedded cameras. By fusing these two pose estimates, the robots can adjust their velocities to achieve more accurate localization and movement.

The final phase involves a demonstration of local path planning and inter-robot communication. Two Neato robots will start at a designated point at the start line and race to a pre-defined end point, initially following hard-coded paths designed to provoke a collision. A subsequent implementation, once each robot has updated the shared map with their predicted path, introduces an adjusted path planner. Each robot will then dynamically prioritize actions based on a hierarchy system, deciding whether to stop, wait, or replan its trajectory to avoid collisions while maintaining efficient navigation.

This work contributes to the study of autonomous robotic systems, highlighting the integration of sensor fusion, local path planning, and multi-agent communication for safe and coordinated fleet operations.

# Timeline:
![alt text](diagram.png)


### Milestone 1


#### What We've Done:

11/18:

- Got launch file that brings up four Neatos and has them follow a routine
- Made node network architecture diagram
- Researched literature on pathplanning and visual odom
- Found library for Kalman filter and started setting up sensor fusion node
- Found script for visual odometry and started cutting it down to essentials

11/20:

- Started working on camera calibration, but the node is not calibrating properly
- Continued working on visual odometry (vivian fill in)
- Continued working on launch file, created a second draft to test

