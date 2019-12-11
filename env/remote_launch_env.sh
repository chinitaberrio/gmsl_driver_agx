#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:/usr/lib/aarch64-linux-gnu
source /opt/ros/kinetic/setup.bash
source /home/nvidia/projects/gmsl_ws/devel/setup.bash


export ROS_IP=10.42.0.28
export ROS_MASTER_URI=http://10.42.0.29:11311
export ROS_HOSTNAME=10.42.0.28

exec "$@"