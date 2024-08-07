

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    auto_mapper = Node(
        package='path_planning',
        executable='auto_mapper',
        name='auto_mapper_node'
        
    )

    path_planner = Node(
        package='path_planning',
        executable='path_planner',
        name='path_planner_node'
        
    )

    

    ld.add_action(auto_mapper)
    ld.add_action(path_planner)

    return ld
