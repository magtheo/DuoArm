#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import json
import numpy as np
import time

class Mapper(Node):
    def __init__(self):
        super().__init__('mapper')

        # Subscriptions and publishers
        self.state_subscription = self.create_subscription(
            String, 'action_controller_state', self.state_callback, 10)
        
        self.button_press_sub = self.create_subscription(
            String, 'map_button_press', self.button_press_callback, 10)
        
        self.joint_angles_pub = self.create_publisher(
            Float64MultiArray, 'joint_angles', 10)
        
        self.manual_readings_pub = self.create_publisher(
            Float64MultiArray, 'manual_angle_readings', 10)
        
        self.set_null_points_pub = self.create_publisher(
            String, 'limp_and_reset_origin', 10)
        
        self.read_angels_pub = self.create_publisher(
            String, 'read_angles', 10)
        
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray, 'actual_joint_angles', self.joint_angles_callback, 10)
        
                # Publishing mapping completion notification
        self.mapping_done_pub = self.create_publisher(
            String, 'mapping_done', 10)
        
        self.joints = {'left': 0, 'right': 0}
        self.mapping_data = {}
        self.button_press_index = 0

    def state_callback(self, msg):
        if msg.data == 'map':
            self.start_mapping()

    def start_mapping(self):
        # Make the motors limp
        self.set_null_points_pub('start')
        time.sleep(5) 
        self.get_logger().info("--- move the arm to top position and press map button to record max angels. --- ") 


    def button_press_callback(self, msg):
        self.button_press_index += 1
        if self.button_press_index == 1: # angles at top position
            self.read_angels_pub()
        if self.button_press_index == 2: # angles at bottom position
            self.read_angels_pub()

    def joint_angles_callback(self, msg):
        self.actual_joint_angles = msg.data
        lss0_angle = self.actual_joint_angles[0]
        lss1_angle = self.actual_joint_angles[1]
        
        if self.button_press_index == 1: # angles at top position
            self.mapping_data['boundaries'] = {'top_lss0': lss0_angle, 'top_lss1': lss1_angle}
            self.get_logger().info(f'Received and mapped boundaries for top position, angles: {(self.actual_joint_angles)}')
            self.get_logger().info(f'--- move arm to bottom position and press map button --- ')
            return

        if self.button_press_index == 2: # angles at bottom position
            self.mapping_data['boundaries'] = {'bottom_lss0': lss0_angle, 'bottom_lss1': lss1_angle}
            self.get_logger().info(f'Received and mapped boundaries for bottom position, angles: {(self.actual_joint_angles)}')
            self.get_logger().info(f'--- move arm to first point in path and press map button --- ')

            return

        if self.button_press_index == 3:
            self.mapping_data['path_point'] = {'nr': 1, 'lss0_angle': lss0_angle, 'lss1_angle': lss1_angle }
            self.get_logger().info(f'Received and mapped angles for point nr 1, angles: {(self.actual_joint_angles)}')
            self.get_logger().info(f'--- move arm to second point in path and press map button --- ')
            return

        if self.button_press_index == 4:
            self.mapping_data['path_point'] = {'nr': 2, 'lss0_angle': lss0_angle, 'lss1_angle': lss1_angle }
            self.get_logger().info(f'Received and mapped angles for point nr 2, angles: {(self.actual_joint_angles)}')
            self.get_logger().info(f'--- move arm to third point in path and press map button --- ')
            return

        if self.button_press_index == 5:
            self.mapping_data['path_point'] = {'nr': 3, 'lss0_angle': lss0_angle, 'lss1_angle': lss1_angle }
            self.get_logger().info(f'Received and mapped angles for point nr 3, angles: {(self.actual_joint_angles)}')
            self.get_logger().info(f'--- all path points mapped --- ')
            # self.get_logger().info(f'--- move arm to forth point in path and press map button --- ')
            self.mapping_done_pub.publish(String(data="done"))  # Notify that mapping is done
            return



    def save_to_file(self):
        with open('path_mapping.json', 'w') as file:
            json.dump(self.mapping_data, file, indent=4)
        self.get_logger().info("Saved mapping data to 'path_mapping.json'.")

def main(args=None):
    rclpy.init(args=args)
    mapper = Mapper()
    rclpy.spin(mapper)
    mapper.save_to_file()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
