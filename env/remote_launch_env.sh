#!/bin/bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:/usr/lib/aarch64-linux-gnu
source /home/projects/gmsl_ws/devel/setup.bash

export ROS_HOSTNAME="xA"

exec "$@"
