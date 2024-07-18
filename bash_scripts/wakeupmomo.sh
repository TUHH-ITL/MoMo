#!/bin/bash

t1="ssh momo@192.168.1.1"  # change ip address (of NUC) if different
t2="ssh momo@192.168.1.1"
t3="ssh momo@192.168.1.1"
t4="source /opt/ros/humble/setup.bash; ros2 launch teleop_twist_joy teleop-launch.py"

gnome-terminal -- bash -c "$t1; exec bash"
gnome-terminal -- bash -c "$t2; exec bash"
gnome-terminal -- bash -c "$t3; exec bash"
gnome-terminal -- bash -c "$t4; exec bash"

