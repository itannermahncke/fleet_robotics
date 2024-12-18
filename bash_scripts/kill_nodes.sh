#!/bin/bash
source ~/ros2_ws/install/setup.bash;


ros2 node list | while read node; do
    #kill $(pgrep -f "$node")
    echo $node
    ros2 lifecycle set $node shutdown
done
