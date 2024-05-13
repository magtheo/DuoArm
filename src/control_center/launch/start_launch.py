# start:launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Control center
    command_window_node = Node(
        package='control_center',
        executable='command_window',
        name='command_window_node'
    )
    command_executor_node = Node(
        package='control_center',
        executable='command_executor',
        name='command_executor_node'
    )
    action_controller = Node(
        package='control_center',
        executable='action_controller',
        name='action_controller'
    )

    # Display
    display_node = Node(
        package='control_center',
        executable='display',
        name='display_node'
    )


    # Mapping and movement
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

    hardware_interface_controller= Node(
        package='hardware_center',
        executable='hardware_interface_controller',
        name='hardware_interface_controller'
    ) 
    gpio_controller= Node(
        package='hardware_center',
        executable='gpio_controller',
        name='gpio_controller'
    ) 
    
    motor_controller = Node(
        package='hardware_center',
        executable='motor_control',
        name='motor_control'
    )
    mapper = Node(
        package='map_and_path',
        executable='mapper',
        name='mapper'
    )

    path = Node(
        package='map_and_path',
        executable='path',
        name='path'
    )
    gui_buttons = Node(
        package='control_center',
        executable='gui_buttons',
        name='gui_buttons'
    )

    #ld.add_action(auto_mapper)
    #ld.add_action(path_planner)
    ld.add_action(action_controller)
    ld.add_action(motor_controller)
    ld.add_action(mapper)
    ld.add_action(path)    
    #ld.add_action(gui_buttons) 
    #ld.add_action(command_window_node)
    #ld.add_action(command_executor_node)
    # ld.add_action(display_node)
    ld.add_action(hardware_interface_controller)
    ld.add_action(gpio_controller)



    return ld
