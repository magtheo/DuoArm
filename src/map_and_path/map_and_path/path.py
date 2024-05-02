#!/usr/bin/env python3
# path.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import json
import numpy as np

class PathExecutor(Node):
    def __init__(self):
        super().__init__('path_executor')

        # Subscription to the state controller
        self.state_subscription = self.create_subscription(
            String, 'action_controller_state', self.state_callback, 10)

        # Publisher to send joint angles
        self.joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 'joint_angles_array', 10)

    def state_callback(self, msg):
        if msg.data == 'path':
            self.get_logger().info("Path execution state activated.")
            self.execute_path()

    def execute_path(self):
        # Load joint angles from the JSON file
        try:
            with open('path_mapping.json', 'r') as file:
                mapping_data = json.load(file)
            self.get_logger().info("Loaded mapping data successfully.")
        except FileNotFoundError:
            self.get_logger().error("File 'path_mapping.json' not found.")
            return
        
        # Collect all angles in a single array
        all_angles = []
        for key, data in mapping_data.items():
            if 'path_point' in key:
                angles = [data['lss0_angle'], data['lss1_angle']]
                all_angles.extend(angles)
        
        if all_angles:
            angles_array = Float64MultiArray(data=all_angles)
            self.joint_angles_publisher.publish(angles_array)
            self.get_logger().info(f"Sent all joint angles for the path to motor controller.")
        else:
            self.get_logger().info("No angles found for path execution.")

def main(args=None):
    rclpy.init(args=args)
    executor = PathExecutor()
    rclpy.spin(executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
