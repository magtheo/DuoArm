#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch controll_center start_launch.py