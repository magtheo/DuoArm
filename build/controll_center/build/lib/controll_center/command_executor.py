#!/usr/bin/env python3
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

        # start test
        self.start_test_pub = self.create_publisher(String, 'start_test', 10)
        
        # start read_angles
        self.start_read_pub = self.create_publisher(String, 'start_read', 10)

        #set new nullpoint
        self.limp_and_reset_origin_pub = self.create_publisher(String, 'limp_and_reset_origin', 10)


    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info('Received command: "%s"' % command)
        # Handle the command message
        self.process_command(command)

    def process_command(self, command):
        if command == 'start_path':
            result = subprocess.run(["ros2", "launch", "path_planning", "path_planning_launch.py"],
                capture_output=True, text=True, shell=True)
            print("Launching path_planner:", result.stdout, result.stderr)
        
        if command == 'start_map':
            self.start_mapping_pub.publish(String(data="start"))
            print("mapping command executed")

        if command == 'test':
            self.start_test_pub.publish(String(data='start'))

        if command == 'read':
            self.start_read_pub.publish(String(data='start'))

        if command == 'set_null':
            new_origin_offset = '0'  # Example offset, change as needed
            self.limp_and_reset_origin_pub.publish(String(data=new_origin_offset))
            print("Limp and reset origin command executed with offset:", new_origin_offset)


        
        
        elif command == 'stop_navigation':
            os.system("ros2 service call /navigation_node/some_service std_srvs/srv/Trigger '{}'")  # Example of stopping a node or action
    


def main(args=None):
    rclpy.init(args=args)
    command_executor = CommandExecutor()
    rclpy.spin(command_executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
