# Visual Odometry and Sensor Fusion

A decentralized path planning system requires fleet members to communicate where they are in the world with a very high degree of accuracy.
Because of this, our team investigated a significant amount of research and development time into the state estimation portion of this project.
We explored two main avenues: visual odometry for high-quality state estimation, and implementing a filter to smooth and fuse sensor data.

## Wheel Odometry

While our team was developing more complex state estimation software, we used the robot's native wheel odometry to construct rudimentary pose
estimates so that we could test our path planning. We developed a simple odometry adapter node that took the robot's latest odometry updates and
offset them by the robot's known starting position to transform them into the world frame.

During this time, we came to truly appreciate the important of managing sensor noise and drift with either sensor fusion or smoothing techniques.
Odometry, we found, has a tendency to accumulate error over time. This meant that by using odometry-based pose estimates, we were severely limiting
the physical distance our robot could traverse before it stopped being able to recognize when it had achieved a step or even a goal. Increasing
tolerance for error extended this lifetime at the cost of earlier inaccuracy, since a higher amount of error was acceptable.

All in all, while odometry-based state estimation was a useful stand-in for accurate state estimation, we would never use it in isolation for
any long-term state estimation. However, it could be a useful tool when paired with another sensor that provided intermediate, highly accurate
estimates, such as a GPS system or visual landmark detection.

## Visual Odometry

## Sensor Fusion Via Extended Kalman Filter

We also explored the avenue of sensor fusion, i.e. combining using multiple sources of position data (visual odometry and wheel odometry, in our case)
to develop a highly accurate pose estimate overall. We first considered using the ROS2 robot_localization package, which runs an Extended Kalman Filter
under the hood to fuse an arbitrary amount of data from various sensors. However, we decided that it would be a great learning experience to write an
EKF node ourselves. We chose to use EKF rather than a standard Kalman Filter so that we could encorporate velocity data, which all types of odometry
are capable of providing but also transforms the system into a nonlinear model. While a Kalman Filter can only model a linear state, EKF includes a
first-order linearization step to expand its modelling capability.

We developed this node with the ability to take either wheel odometry or visual odometry, or both, and utilize them to continually improve the filter's
internal model of the world. We also developed a "pose plotter" node that listened to both odometry outputs and the EKF's pose model and compared the
outputted pose estimates. While we ran out of time to fully test the EKF node or integrate it with the rest of the system, writing the filter ourselves
was an immensely educational experience that deepened our understanding of the theory behind sensor fusion techniques.