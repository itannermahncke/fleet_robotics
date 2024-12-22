# Path Planning

We are using a decentralized, online path planning system capable of planning around known obstacles within a discrete world. This means that every robot in
the fleet is doing its own planning locally -- no central computer maps anything out beforehand. To avoid having to track dynamic obstacles, other robots
in the fleet are not considered obstacles to avoid and are instead avoided with a specialized crash handler.

## Research and Design Decisions
Initially, we looked into many path-planning algorithms such as [A\*](https://en.wikipedia.org/wiki/A*_search_algorithm), [Dijkstra's](https://en.wikipedia.org/wiki/Dijkstra's_algorithm), and [D\* Lite](https://en.wikipedia.org/wiki/D*). We made the design decision to internally
represent our world as a discrete grid full of squares that were either empty or occupied. Due to the nature of this project, we assumed that this discrete world is only sparsely populated by obstacles. The algorithm we wrote takes in the robot's current location on the grid and the location of its goal. Then, it calculates the absolute distance between the spaces around the Neato and their distances to the goal. The square with the shortest distance to the robot's goal is selected to be the robot's next "step" in the world. After confirming that our path-planning algorithm works, we decided to implement obstacle avoidance by adding a check when calculating the next robot step to see if it is occupied by a predefined obstacle.

Below is a video of our path planning algorithm in action on a single robot, who's goal is to reach the top right part of the map without crashing into any obstacles:

<iframe width="560" height="315" src="https://www.youtube.com/embed/l7JHn1P49_8?si=QmKHv_wa8TMfYRXa" title="Single Neato Path Planning" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Since other fleet members are not considered obstacles (due to their dynamic nature) we designed a separate node in the ROS2 network called the crash handler.
The crash handler is network-facing, meaning it listens to every fleet member's broadcasts about A) their current position, and B) the next step they plan to
take. Every robot's local crash handler will compare these values to it's own robot's position and planned step, and evaluate any potential crashes. We defined
two types of crashes, each with different behavior:

<img src="assets/IMG_0057.jpg" alt="Image" width="700px">

The crash handler critically relies on the different fleet members agreeing on who gets priority in a crash avoidance event. To simplify things, we predefined
a right-of-way heirarchy for the fleet that all robots abided to, which negated the need for a "confirmation" step in which the robot's communicated their opinions to each other. Our crash handling system can be seen in action in this video, where the lower priority white robot pauses all motion until the higher priority robot is completely out of the way:

<iframe width="560" height="315" src="https://www.youtube.com/embed/cGe1KRgT8iM?si=bJx4UclFSttoG1Ti" title="Fleet Working Demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Future Steps

Our stretch goal for path planning is to write an additional obstacle detection node using the Neato's LiDAR sensor. We would take note of detected LiDAR obstacles, filtering out obstacles that seem to be fleet members (based on communicated fleet member positions), and broadcast them to a shared map. This way, one robot could detect an obstacle, and all other robots could instantly account for it in their own path planning.

We would also have liked to implement a confidence level for obstacle detection: if more than one robot detects the same obstacle in the same place, the confidence level for that obstacle would be high. If two robots provide conflicting information (e.g., one Neato reports a grid as occupied while another reports it as clear), the confidence level would be low. In the final shared map, we would set a threshold for the confidence level so that only obstacles with high confidence are included.

Another stretch goal would be to shift from a predefined heirarchy to an online, in-the-moment determination of right-of-way. One way this could look is with random choice, i.e. both robots generate a number and communicate that to each other over the network, and whoever has the higher number gets to go. This would
also need a second "confirmation" step in which the robots ensure they both agree on who has right-of-way, this would avoid potential crashes caused by network
lag or slow computational decision-making.
