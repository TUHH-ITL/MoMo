#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ros1_bridge dynamic_bridge
