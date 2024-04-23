# start:launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Controll center
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

    # Display
    display_node = Node(
        package='controll_center',
        executable='display',
        name='display_node'
    )


    # Mapping and movment
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

    # Joystick 
    joystick_controller = Node(
        package='controll_center',
        executable='joystick_controller',
        name='joystick_controller'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )    

    # motor controller
    motor_controller = Node(
        package='motor_controller',
        executable='motor_control',
        name='motor_controller'
    )

    

    ld.add_action(auto_mapper)
    #ld.add_action(path_planner)
    ld.add_action(action_controller)
    ld.add_action(motor_controller)
    

    ld.add_action(command_window_node)
    ld.add_action(command_executor_node)
    ld.add_action(display_node)

    # ld.add_action(joystick_controller)
    # ld.add_action(joy_node)


    return ld
