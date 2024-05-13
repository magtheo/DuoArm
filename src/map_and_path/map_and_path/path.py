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
        
        self.init_communication()

    def init_communication(self):
        """
        Initialize communication for the node.
        Setup all necessary publishers and subscribers for sending and receiving commands
        and data to and from other components of ROS system.
        """
        # Subscription to the state
        self.state_subscription = self.create_subscription(
            String, 'system_state', self.state_callback, 10)

        # Publisher to send joint angles to motorController for path execution
        self.joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 'joint_angles_array', 10)

    def state_callback(self, msg):
        """React to state changes by checking if the predefined path needs to be run and, if so, executing it."""
        if msg.data == 'run_predefined_path':
            self.get_logger().info("Path execution state activated.")
            self.execute_path()

    def execute_path(self):
        """
        Execute the path as per the predefined joint angles. This involves:
        - Loading the path points from a JSON file.
        - Sending the joint angles to the motor controller for execution.
        If no angles are found, it logs the absence of data.
        """
        # Load joint angles from the JSON file
        try:
            with open('boundary_path_and_rail_position.json', 'r') as file:
                mapping_data = json.load(file)
            self.get_logger().info("Loaded mapping data successfully.")
        except FileNotFoundError:
            self.get_logger().error("File boundary_path_and_rail_position.json not found.")
            return
        
        # Collect all angles in a single array
        all_angles = []
        for point in mapping_data.get('path_points', []):  # safely access path_points
            angles = [point['lss0_angle'], point['lss1_angle']]
            all_angles.extend(angles)
        
        if all_angles:
            angles_array = Float64MultiArray()
            angles_array.data = all_angles
            self.joint_angles_publisher.publish(angles_array)
            self.get_logger().info("Sent all joint angles for the path to motor controller.")
        else:
            self.get_logger().info("No angles found for path execution.")


def main(args=None):
    """Main function to initialize the ROS node, spin the PathExecutor, and handle shutdown."""
    rclpy.init(args=args)
    executor = PathExecutor()
    rclpy.spin(executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()