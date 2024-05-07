#!/bin/bash
# build_and_launch_startup.bash

cd Documents/DuoArm/

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch control_center start_launch.py
