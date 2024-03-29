

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    path_planner = Node(
        package='path_planning',
        executable='path_planner',
        name='path_planner_node'
    )

    ld.add_action(path_planner)

    return ld
