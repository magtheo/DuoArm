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
        self.create_subscription(String, 'mapping_done', self.mapping_done_callback, 10)
        self.create_subscription(String, 'path_done', self.path_done_callback, 10)
        self.create_subscription(String, 'state_change', self.handle_state_change, 10)

    def handle_state_change(self, msg):
        """Handles incoming requests to change the state."""
        new_state = msg.data
        if new_state != self.state:
            self.state = new_state
            self.publish_state()
            self.get_logger().info(f'State updated to: {self.state}')

    def publish_state(self):
        # This method publishes the state when it changes
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)
        self.get_logger().info(f'Published state: {msg.data}')


    def mapping_done_callback(self, msg):
        if msg.data == 'done':
            self.state = 'standby'
            self.publish_state()
            self.get_logger().info('Mapping completed | Returning to standby state.')

    def path_done_callback(self, msg):
        if msg.data == 'done':
            self.state = 'standby'
            self.publish_state()
            self.get_logger().info(' Path completed | Returning to standby state.')

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
