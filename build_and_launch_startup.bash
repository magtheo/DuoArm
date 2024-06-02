#!/bin/bash

cd Documents/DuoArm/

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch control_center start_launch.py
