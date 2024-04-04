#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class state_controller(Node):
    def __init__(self):
        super().__init__('action_controller')
        self.state = 'user'

        # Subscribe to the topic for mode change
        self.mode_subscription = self.create_subscription(
            String,
            'mode_change',
            self.handle_mode_change,
            10)
            
        # Subscribe to the topic for action button press
        self.action_subscription = self.create_subscription(
            String,
            'action_button',
            self.handle_action_button,
            10)

    def handle_mode_change(self, msg):
            if msg.data in ['user', 'admin']: # TODO bytt til en user
                self.get_logger().info(f'Switching to {msg.data} mode.')
                self.state = msg.data
            else:
                self.get_logger().warning('Received invalid mode change command.')

    def handle_action_button(self, msg):
        if msg.data == 'pressed':
            if self.state == 'admin':
                self.get_logger().info('Admin mode: Running mapping function...')
                # Run the mapping function
                self.run_mapping()
            elif self.state == 'user':
                self.get_logger().info('User mode: Starting predefined path...')
                # Start the predefined path
                self.start_predefined_path()
        else:
            self.get_logger().warning('Received invalid action button command.')

    def run_mapping(self):
        # TODO: initiate mapping
        pass

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
