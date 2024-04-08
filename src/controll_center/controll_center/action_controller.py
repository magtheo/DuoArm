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
        self.start_mapping_pub = self.create_publisher(String, 'start_mapping', 10)


        ### Buttons    
        # Subscribe to the topic for predefined_path button press
        self.action_subscription = self.create_subscription(
            String,
            'predefined_path_button',
            self.handle_predefined_path_button,
            10)
        
        # Subscribe to the topic for action button press
        self.action_subscription = self.create_subscription(
            String,
            'map_button',
            self.handle_map_button,
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

        if self.state == 'predefined_path':
            self.start_predefined_path()


    def handle_predefined_path_button(self, msg):
        if msg.data == 'pressed':
            self.state = 'predefined_path'
            self.get_logger().info('Starting predefined path')
            # Start the predefined path
        else:
            self.get_logger().warning('Received invalid button command.')

    def handle_map_button(self, msg):
        if msg.data == 'pressed':
            self.state = 'map'
            self.get_logger().info('Starting mapping')
            self.start_mapping_pub.publish(String(data="start"))
        else:
            self.get_logger().warning('Received invalid button command.')


    def start_predefined_path(self):
        # TODO: initate predefined path
        pass

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
