

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    auto_mapper = Node(
        package='path_planning',
        executable='auto_mapper',
        name='auto_mapper_node'
        
    )

    ld.add_action(auto_mapper)

    return ld
