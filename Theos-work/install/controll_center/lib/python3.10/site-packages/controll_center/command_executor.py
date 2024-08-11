#!/usr/bin/env python3
# command_executor

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import subprocess


class CommandExecutor(Node):
    def __init__(self):
        super().__init__('command_executor')
        self.subscription = self.create_subscription(
            String,
            'command_topic',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publisher for starting mapping
        self.start_mapping_pub = self.create_publisher(String, 'command_start_mapping', 10)

        # publisher for state changes
        self.state_change_pub = self.create_publisher(String, 'state_change', 10)

        # start test
        self.start_test_pub = self.create_publisher(String, 'start_test', 10)
        
        # start read_angles
        self.start_read_pub = self.create_publisher(String, 'start_read', 10)

        # start path
        self.start_path_pub = self.create_publisher(String, 'start_path', 10)

        #set new nullpoint
        self.limp_and_reset_origin_pub = self.create_publisher(String, 'limp_and_reset_origin', 10)

    def set_state(self, state):
        """Publish a command to change the state of the action controller."""
        msg = String()
        msg.data = state
        self.state_change_pub.publish(msg)
        self.get_logger().info(f'Command sent to change state to: {state}')

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Received command: "%s"' % command)
        # Handle the command message
        self.process_command(command)

    def process_command(self, command):
        if command == 'path': # TODO set arm state to path
            self.set_state("path")
        
        if command == 'start_map':
            self.set_state("map")

        if command == 'test':
            self.start_test_pub.publish(String(data='start'))

        if command == 'read':
            self.start_read_pub.publish(String(data='start'))

        if command == 'set_null':
            self.limp_and_reset_origin_pub.publish(String(data='start'))
            self.get_logger().info(f"Limp and reset origin command executed ")

        elif command == 'stop_navigation':
            os.system("ros2 service call /navigation_node/some_service std_srvs/srv/Trigger '{}'")  # Example of stopping a node or action
    


def main(args=None):
    rclpy.init(args=args)
    command_executor = CommandExecutor()
    rclpy.spin(command_executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
