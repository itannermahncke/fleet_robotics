#!/bin/bash
source ~/ros2_ws/install/setup.bash;


ros2 node list | while read node; do
    kill $(pgrep -f "$node")
done
