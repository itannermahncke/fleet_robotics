---
title: Home
layout: default
filename: index.md
--- 
# Fleet Robotics
## Fall 2024: Intro to Computational Robotics Final Project
### Contributors: Ivy, Vivian, Ariel, Charlie

# Abstract
This project aims to develop and implement a control system for a fleet of robots, integrating sensor fusion techniques, and multi-agent communication for autonomous fleet navigation. 

The system combines internal odometry data from Neato robots (using ROS 2) with visual odometry derived from embedded cameras. By fusing these two pose estimates, the robots can adjust their velocities to achieve more accurate localization and movement.

The final product involves a demonstration of a decentraized path planning algortihm and inter-robot communication. Two Neato robots will start at a designated point at the start line and race to a pre-defined end point, initially following hard-coded paths designed to provoke a collision. A subsequent implementation, once each robot has updated the shared map with their predicted path, introduces an adjusted path planner. Each robot will then dynamically prioritize actions based on a hierarchy system, deciding whether to stop, wait, or replan its trajectory to avoid collisions while maintaining efficient navigation.
