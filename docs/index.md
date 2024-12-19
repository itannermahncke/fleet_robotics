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

<iframe width="560" height="315" src="https://www.youtube.com/embed/cGe1KRgT8iM?si=bJx4UclFSttoG1Ti" title="Fleet Working Demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/b3jujHLeyG4?si=SKKd6wZU1L0yHhNd" title="Fleet Fail #1" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


<iframe width="560" height="315" src="https://www.youtube.com/embed/l7JHn1P49_8?si=QmKHv_wa8TMfYRXa" title="Single Neato Path Planning" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
