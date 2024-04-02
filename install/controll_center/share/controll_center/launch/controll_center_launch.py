
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

    ld.add_action(command_window_node)
    ld.add_action(command_executor_node)

    return ld
