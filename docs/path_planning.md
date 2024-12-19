# Path Planning

Initially, we looked into many path-planning algorithms such as A*, Dijkstra's, and D* Lite. We made design decisions to represent our map using discrete grids. Due to the nature of this project, we assumed that the discrete world the Neatos are traversing is neither complicated nor densely populated with obstacles. Given this, we decided to simplify the algorithm to one that intakes the Neato's current grid and the goal grid. Then, it calculates the grids around the Neato and their distances to the goal, selecting the grid with the minimum distance to the goal as the next grid. the next grid then is published to the motion execution node where the turn velocity and linear velocity is published to the neato. 

After confirming that our path-planning algorithm works, we decided to implement obstacle avoidance by adding a check when calculating the next grid to see if it is occupied by an obstacle.

Another aspect of the path planning is how potential crash between neatos are handled. 

## Future Steps

Our stretch goal for path planning is to write an additional obstacle detection node using the Neato's LiDAR sensor. Another stretch goal is for the individual Neatos to maintain a local map of the world and share the locations of detected obstacles, ultimately generating a world map with all obstacles.

We would implement a confidence level: if more than one Neato detects the same obstacle in the same place, the confidence level for that obstacle would be high. If two Neatos provide conflicting information (e.g., one Neato reports a grid as occupied while another reports it as clear), the confidence level would be low. In the final shared world map, we would set a threshold for the confidence level so that only obstacles with high confidence are included.

Additionally, we considered that the LiDAR might detect other Neatos as obstacles. To address this, we would set a threshold by comparing the other Neato's pose. If the obstacle data points align with the grid coordinates of another Neato, we would not consider it an obstacle.


