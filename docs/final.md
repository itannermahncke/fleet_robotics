# Milestone 3 (Final System)

For the final milestone, we focused on application and integration. This means we deeply fleshed our out network communication system, implemented our path planner and crash handler on real robots, and pushed forward on our work related to visual odometry and sensor fusion. Finally, we created this system diagram
to show off the full scope of our project:

![A system diagram of a single Neato's node network interfacing with the broader Neato network. The local network is split up into two sections: state estimation and path planning.](m3_diagram.png)


## Confirming Network Communication

For the final milestone, we fully developed the network_startup node, which performs two functions: A) confirm good communication between the local robot and the rest of the fleet, and B) calculate time offsets between the local robot's system time and the rest of the fleet's system times. This was greatly beneficial
to our crash handler, which could actually utilize the functionality of discarding stale data when planning around other fleet members' actions.

We did run into some challenges related to remapping the nodes and topics in ROS2 to be unique to each local robot, while still being able to maintain inter-fleet communications. In particular, we found that local subcribers that were meant to be listening to the wider fleet network were getting their topics overwritten and therefore unable to perform their role. We got around this by finding a sneaky way to prevent a topic from being remapped despite belonging to a node within a namespace, and this allowed our network_startup and crash_handler node to perform their roles well.

We also ran into challenges related to passing each robot's name as a command-line argument to its own launch file in a for loop within the bash script. Essentially, we discovered that it is very difficult to access the actual value of a launch argument in a ROS2 launch file. We did not have time to find an elegant solution to this, so we instead created separate launch files for each robot that were identical besides a "robot_name" variable that was hard-coded. While this is an unfortunately inelegant solution, it allows users to continue to launch the entire fleet using a single bash script, which was the most important feature within the launch tasks.

## Path Planning

Our progress on the path planning portion of the project was highly successful in the final two weeks. We successfully demonstrated our shortest-path method on a single robot at first (relying temporarily on wheel odometry-based pose estimates). When this worked, we added in obstacle avoidance capabilities and predefined obstacles on the robot's reference map. Finally, when the launch system was completed, we tested the path planning and specifically the crash handling on a fkeet of two robots. Because we performed so much testing with a single robot, we were able to debug a lot of our code before upgrading to fleetwide path planning; this made the job of debugging multiple robots significantly simpler.

## Visual Odometry



## Sensor Fusion

Since our team had some free time at the end of this milestone, we decided to explore the mathematical implementation of an Extended Kalman Filter. We developed a ROS2 node wrapper for EKF functionality that could receive odometry data from any source (wheel odometry or visual odometry) and use it as the observation with which to update its predictions about the state of the robot's position and velocity. We specifically went with an EKF rather than a simple Kalman Filter, because an EKF includes a linearizing step that allows your model to make predicitons about nonlinear systems, i.e. positon AND velocity. While we never got the chance to fully integrate the EKF into our wider state estimation system, it was immensely educational to try out the implementation ourselves and gain a research-based understanding of why filtering and sensor fusion are so critical for state estimation (and especially position estimation).