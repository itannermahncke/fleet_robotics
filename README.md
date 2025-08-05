# Decentralized Path Planning For A Robotic Fleet
## Overview
This project was created for the Computational Robotics F'24 Final.

This project aims to develop and implement a decentralized path planning system for an arbitrarily-sized fleet of robots. We integrate sensor fusion techniques, multi-agent communication, and greedy path planning for a fully autonomous fleet navigation system.

The final product involves a demonstration of a decentralized path planning algorithm and inter-robot communication. Two Neato robots will start at designated points at the start line and race to unique pre-defined end points that are designed to provoke a collision. The robots continually communicate their current positions in the world and their planned next steps with each other, and independently detect crashes. When a crash event is anticipated, the robots will locally decide whether to stop or go, confirm that they agree, and act upon it. At the end of the demo, both robots reach their goals without colliding!

To read more about the development and technical details of our project, please view the [project website.](https://itannermahncke.github.io/fleet_robotics/)

## Directory Structure
### fleet_robotics/
Source code. Each file defines a node found in each robot's ROS2 network. This includes the following functionality:
- Motion control/waypoint following `(motion_execution.py)`
- Communication between agents `(network_startup.py)`
- Multi-agent crash handling and avoidance `(crash_handling.py)`
- D* path planning `(path_no_obstacle.py)`
- State estimation via...
    - Dead reckoning `(odom_adapter.py)`
    - Visual odometry `(visual_odometry.py)`

As well as some unfinished stretch goals:
- State estimation via Kalman Filtering `(extended_kalman_filter.py)`
- Static obstacle detection `(obstacle_detection.py)`

And finally, calibration data for visual odometry is found in `visual_odom/`.

### launch/
Contains launchfiles for starting up individual robotic agents, as well as several launch files to help test specific parts of the overall system. To start the multi-agent fleet, do not use launchfiles; instead use our bash scripts.

### config/
Configuration files used by launchfiles to run with different types of fleets and assumptions about the world.

### bash_scripts/
Scripts for launching and killing the fleet all together.

### docs/
Website documentation files.

## Installation and Execution
This package is designed for use on the Olin Neato robots; other robotic platforms may be compatible. To run on other platforms, be sure that the topic names your platform is listening to match up with that of the source code.

First, follow the [ROS2 Humble installation and setup instructions.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Next, install the dependencies listed in `requirements.txt.`

Finally, clone the following ROS2 packages into your ROS2 workspace:
- https://github.com/comprobo24/neato_packages (if using Olin Neatos)
- https://github.com/itannermahncke/fleet_robotics_msgs

To execute the code, make sure that your computer is on the same network as all of the fleet members. Then, execute bash_scripts/fleet_launch.sh or use launch/fleet_member.launch.py for each member of the fleet.
