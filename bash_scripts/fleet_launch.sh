#!/bin/bash

source ~/ros2_ws/install/setup.bash;

# This script assumes the following arguments: {number of robots n} \
# {ip address of robot 1} ... {ip address of robot n)
# Example: ./fleet_launch.sh 2 192.168.16.132 192.168.17.40

# first, save the number of robots to a variable
num_bots=$1
ip_list=()
# next, shift the starting point of the arguments 1 to the right
shift 1 # now $1 refers to the first IP address

# for each item remaining in the argument list
for ip in "$@"
do
    ip_list+=($ip) # append the current IP address to the ip list
    shift 1 # shift the starting point one to the right -- the next IP
done

#initializes lists for all robot params
robot_names=()
video_ports=()
sensor_ports=()
launch_pid=()


#sets the names and ports for each of the robots
for ((i=1; i<=num_bots; i++))
do
    robot_names+=(robot$i)
    video_ports+=(500$i)
    sensor_ports+=(777$i)
done
#created the 'gscam_config' part of the launch
declare -i iter=0
gscam_config_p1="'udpsrc port="
gscam_config_p2=" ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'"
#for each robot, this will combine the final gscam arg and then launch the neato
for ip in "${ip_list[@]}"
do
    gscam_config=${gscam_config_p1}${video_ports[$iter]}${gscam_config_p2}; echo ${gscam_config}
    echo $(gscam_config)
    (ros2 launch neato_node2 bringup_multi.py host:=$ip robot_name:=${robot_names[iter]} udp_video_port:=${video_ports[iter]} udp_sensor_port:=${sensor_ports[iter]} gscam_config:="${gscam_config}") &
    (ros2 launch fleet_robotics fleet_member.launch.py robot_name:=${robot_names[iter]})
    launch_pid[iter]=$!
    ((iter++))
    echo "........................................................................................................................................"
    echo ${launch_pid[@]}
    echo "........................................................................................................................................"
done
