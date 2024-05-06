#!/usr/bin/env python3
# mapper.py
import rclpy
from rclpy.node import Node
<<<<<<< HEAD
from std_msgs.msg import String, Float64MultiArray
=======
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
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
<<<<<<< HEAD
            String, 'map_button_press', self.button_press_callback, 10)
=======
            String, 'map_button_pressed', self.button_press_callback, 10)
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
        
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
        
<<<<<<< HEAD
                # Publishing mapping completion notification
=======
        # Publishing mapping completion notification
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
        self.mapping_done_pub = self.create_publisher(
            String, 'mapping_done', 10)
        
        self.joints = {'left': 0, 'right': 0}
<<<<<<< HEAD
        self.mapping_data = {}
        self.button_press_index = 0

    def state_callback(self, msg):
        if msg.data == 'map':
            self.start_mapping()

    def start_mapping(self):
        # Make the motors limp
        self.set_null_points_pub('start')
=======
        self.mapping_data = {'boundaries': [], 'path_points': []}
        self.button_press_index = 0
        self.number_of_points = 3
        self.point_number = 0

        self.state = 'standby'

    def state_callback(self, msg):
        if msg.data == 'map' and self.state != 'map':
            self.state = 'map'
            self.start_mapping()

        if msg.data == 'standby':
            self.button_press_index = 0
            self.point_number = 0


    def start_mapping(self):
        # Make the motors limp
        self.set_null_points_pub.publish(String(data='start'))
        self.get_logger().info('start map')
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a
        time.sleep(5) 
        self.get_logger().info("--- move the arm to top position and press map button to record max angels. --- ") 


    def button_press_callback(self, msg):
<<<<<<< HEAD
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
=======
        self.get_logger().info("Recived button press in mapper node ") 
        self.button_press_index += 1
        self.read_angels_pub.publish(String(data='start'))

    def joint_angles_callback(self, msg):
        try:
            angles = msg.data  # Assuming this is already an array of float64

            # Set layout
            float_array = Float64MultiArray()
            dim = MultiArrayDimension()
            dim.label = "joint_angles"
            dim.size = len(angles)
            dim.stride = len(angles)
            float_array.layout.dim.append(dim)
            float_array.data = angles
            self.joint_angles_pub.publish(float_array)

            self.get_logger().info("Published joint angles for mapping.")
            lss0_angle = angles[0]
            lss1_angle = angles[1]
            
            # Append new data based on button press index
            if self.button_press_index == 1:  # Top position boundaries
                self.mapping_data['boundaries'].append({'top_lss0': lss0_angle, 'top_lss1': lss1_angle})
                self.get_logger().info(f'Received and mapped top boundaries, angles: {angles}')
                self.get_logger().info('--- move arm to bottom position and press map button --- ')
            
            elif self.button_press_index == 2:  # Bottom position boundaries
                self.mapping_data['boundaries'].append({'bottom_lss0': lss0_angle, 'bottom_lss1': lss1_angle})
                self.get_logger().info(f'Received and mapped bottom boundaries, angles: {angles}')
                self.get_logger().info('--- move arm to first point in path and press map button --- ')

            elif self.button_press_index >= 3:  # Path points
                self.point_number += 1
                self.mapping_data['path_points'].append({'nr': self.point_number, 'lss0_angle': lss0_angle, 'lss1_angle': lss1_angle})
                self.get_logger().info(f'Received and mapped angles for point nr {self.point_number}, angles: {angles}')
                if self.point_number < self.number_of_points:
                    self.get_logger().info(f'--- move arm to point {self.point_number + 1} in path and press map button --- ')
                else:
                    self.get_logger().info('--- all path points mapped --- ')
                    self.point_number = 0
                    self.button_press_index = 0
                    self.state = 'standby'
                    self.mapping_done_pub.publish(String(data="done"))  # Notify that mapping is done
                    self.save_to_file()  # Save the data to file once all mapping is completed
                    
        except Exception as e:
            self.get_logger().error(f'failed to process joint angels{e}')

    def save_to_file(self):
        with open('boundary_path_and_rail_position.json', 'w') as file:
            json.dump(self.mapping_data, file, indent=4)
        self.get_logger().info("Saved mapping data to boundary_path_and_rail_position.json.")
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a

def main(args=None):
    rclpy.init(args=args)
    mapper = Mapper()
    rclpy.spin(mapper)
    mapper.save_to_file()
    rclpy.shutdown()

if __name__ == '__main__':
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> ff002f6542b223921602b615f31a4889708bc41a