#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ActionController(Node):
    def __init__(self):
        super().__init__('action_controller')
        self.state = 'standby'
            
        ### comunication with mapping node
        # Publisher for starting mapping
        self.start_mapping_publisher = self.create_publisher(
            String,
            'start_mapping',
            10)


        ### Buttons    
        # Subscribe to the topic for predefined_path button press
        self.action_subscription = self.create_subscription(
            String,
            'predefined_path_button',
            self.sett_predefined_path_state,
            10)
        
        # Subscribe to the topic for action button press
        self.action_subscription = self.create_subscription(
            String,
            'command_start_mapping',
            self.sett_map_state_with_command,
            10)
        
        # Subscribe to the mapping_done topic
        self.mapping_done_subscription = self.create_subscription(
            String,
            'mapping_done',
            self.mapping_done_callback,
            10)


    def handle_state(self, ):
        if self.state == 'joystick':
            self.listen_to_joystick()

        elif self.state == 'predefined_path':
            self.start_predefined_path()

        elif self.state == 'map':
            self.get_logger().info('Starting mapping')
            self.start_mapping_publisher.publish(String(data="start"))
        else:
            self.state = 'standby'


    def sett_predefined_path_state(self, msg):
        if msg.data == 'pressed':
            self.state = 'predefined_path'
            self.get_logger().info('Starting predefined path')
            # Start the predefined path
        else:
            self.get_logger().warning('Received invalid button command.')
            
    def sett_map_state_with_command(self, msg):
        if msg.data == 'pressed' and self.state == 'standby':
            self.state = 'map'
            self.get_logger().info('Starting mapping')
            # self.start_mapping_publisher.publish(String(data="start"))


    def start_predefined_path(self):
        # TODO: initate predefined path
        pass
    
    def mapping_done_callback(self, msg):
        if msg.data == 'mapping_done':
            self.state = 'standby'

def main(args=None):
    rclpy.init(args=args)
    action_controller = ActionController()
    
    try:
        rclpy.spin(action_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the ROS client library for Python
        rclpy.shutdown()

if __name__ == '__main__':
    main()
