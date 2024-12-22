---
title: System
layout: default
filename: system.md
--- 
# System Overview

For the general outline of our system, We have a diagram that lays out the communication between sections

<img src="assets/m3_diagram.png" alt="System Diagram" width="1000px">

In this diagram, you can see that there are 3 main sections:
- Path Planning
    - Contains the path planner, Neato Driver, and crash handler
- State Estimation
    - Contains our sensor fusion and Visual odom
- Neato Network Communication
    - Contains the network startup functionality, and inter-fleet message-passing

This software system interfaces with a physical vaccuum robot that we will occasionally refer to as a "Neato", which passes our software system sensor data while receiving motor commands. To learn more about each piece, check out the pages under "System"!