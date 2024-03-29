#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class CommandExecutor(Node):
    def __init__(self):
        super().__init__('command_executor')
        self.subscription = self.create_subscription(
            String,
            'command_topic',
            self.command_callback,
            10)
        self.subscription  # prevent unused variable warning

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Received command: "%s"' % command)
        # Handle the command message
        self.process_command(command)

    def process_command(self, command):
        if command == 'start_path':
            os.system("ros2 launch path_planning path_planning_launch.py")
            print("Launching path_planner")
        elif command == 'stop_navigation':
            os.system("ros2 service call /navigation_node/some_service std_srvs/srv/Trigger '{}'")  # Example of stopping a node or action

def main(args=None):
    rclpy.init(args=args)
    command_executor = CommandExecutor()
    rclpy.spin(command_executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
