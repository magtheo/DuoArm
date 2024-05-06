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
<<<<<<< HEAD
            with open('path_mapping.json', 'r') as file:
                mapping_data = json.load(file)
            self.get_logger().info("Loaded mapping data successfully.")
        except FileNotFoundError:
            self.get_logger().error("File 'path_mapping.json' not found.")
=======
            with open('boundary_and_path.json', 'r') as file:
                mapping_data = json.load(file)
            self.get_logger().info("Loaded mapping data successfully.")
        except FileNotFoundError:
            self.get_logger().error("File 'boundary_and_path.json' not found.")
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
            return
        
        # Collect all angles in a single array
        all_angles = []
<<<<<<< HEAD
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

=======
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


>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
def main(args=None):
    rclpy.init(args=args)
    executor = PathExecutor()
    rclpy.spin(executor)
    rclpy.shutdown()

if __name__ == '__main__':
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
