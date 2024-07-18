#!/bin/bash

# Password for the control PC (Intel NUC) is 1234 (change if different)
echo '1234' | sudo -S ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch momo_description momo_description.launch
