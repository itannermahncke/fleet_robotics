---
title: Home
layout: default
filename: index.md
--- 
# Fleet Robotics
## Fall 2024: Intro to Computational Robotics Final Project
### Contributors: Ivy, Vivian, Ariel, Charlie

# Abstract
This project aims to develop and implement a decentralized path planning system for an arbitrarily-sized fleet of robots.
We integrate sensor fusion techniques, multi-agent communication, and greedy path planning for a fully autonomous fleet navigation system.

The final product involves a demonstration of a decentralized path planning algorithm and inter-robot communication. Two Neato robots will
start at designated points at the start line and race to unique pre-defined end points that are designed to provoke a collision. The robots
continually communicate their current positions in the world and their planned next steps with each other, and independently detect crashes.
When a crash event is anticipated, the robots will locally decide whether to stop or go, confirm that they agree, and act upon it. At the
end of the demo, both robots reach their goals without colliding!

<iframe width="560" height="315" src="https://www.youtube.com/embed/cGe1KRgT8iM?si=bJx4UclFSttoG1Ti" title="Fleet Working Demo" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe width="560" height="315" src="https://www.youtube.com/embed/b3jujHLeyG4?si=SKKd6wZU1L0yHhNd" title="Fleet Fail #1" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>


<iframe width="560" height="315" src="https://www.youtube.com/embed/l7JHn1P49_8?si=QmKHv_wa8TMfYRXa" title="Single Neato Path Planning" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
