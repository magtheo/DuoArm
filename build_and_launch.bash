#!/bin/bash

colcon build

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch launcher start_launch.py