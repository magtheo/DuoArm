#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ActionController(Node):
    def __init__(self):
        super().__init__('action_controller')
        self.state = 'standby'
        

        # Publisher for the current state
        self.state_publisher = self.create_publisher(String, 'action_controller_state', 10)

        # Subscriptions
        self.create_subscription(String, 'predefined_path_button', self.set_predefined_path_state, 10)
        self.create_subscription(String, 'command_start_mapping', self.set_map_state_with_command, 10)
        self.create_subscription(String, 'mapping_done', self.mapping_done_callback, 10)



    def publish_state(self):
        # This method publishes the state when it changes
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Published state: {msg.data}')

    def set_predefined_path_state(self, msg): # TODO needs rework
        if msg.data == 'pressed':
            self.state = 'predefined_path'
            self.publish_state()
            self.get_logger().info('Setting state: Starting predefined path')

    def set_map_state_with_command(self, msg):
        if msg.data == 'start' and self.state == 'standby':
            self.state = 'map'
            self.publish_state()
            self.get_logger().info('state set: map')

    def mapping_done_callback(self, msg):
        if msg.data == 'done':
            self.state = 'standby'
            self.publish_state()
            self.get_logger().info('Setting state: Mapping completed. Returning to standby state.')

def main(args=None):
    rclpy.init(args=args)
    action_controller = ActionController()

    try:
        rclpy.spin(action_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
