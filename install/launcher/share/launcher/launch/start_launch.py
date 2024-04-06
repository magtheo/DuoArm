# start:launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    command_window_node = Node(
        package='controll_center',
        executable='command_window',
        name='command_window_node'
    )
    command_executor_node = Node(
        package='controll_center',
        executable='command_executor',
        name='command_executor_node'
    )
    action_controller = Node(
        package='controll_center',
        executable='action_controller',
        name='action_controller'
    )

    display_node = Node(
        package='controll_center',
        executable='display',
        name='display_node'
    )


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
    ld.add_action(action_controller)

    ld.add_action(command_window_node)
    ld.add_action(command_executor_node)
    ld.add_action(display_node)

    return ld
