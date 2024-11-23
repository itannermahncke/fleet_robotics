#!/bin/bash

source ~/ros2_ws/install/setup.bash;

# This script assumes the following arguments: {number of robots n} \
# {ip address of robot 1} ... {ip address of robot n)
# Example: ./fleet_launch.sh 2 192.168.16.132 192.168.17.40

# first, save the number of robots to a variable
num_bots=$1

# next, shift the starting point of the arguments 1 to the right
shift 1 # now $1 refers to the first IP address

# for each item remaining in the argument list
for ip in "$@"
do
    echo $1 # print the current IP address
    shift 1 # shift the starting point one to the right -- the next IP
done
