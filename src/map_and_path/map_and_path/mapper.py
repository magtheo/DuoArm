#!/usr/bin/env python3
# mapper.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
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
            String, 'map_button_pressed', self.button_press_callback, 10)
        
        self.joint_angles_pub = self.create_publisher( # jointangels published to display
            Float64MultiArray, 'joint_angles', 10)
        
        self.manual_readings_pub = self.create_publisher(
            Float64MultiArray, 'manual_angle_readings', 10)
        
        self.set_null_points_pub = self.create_publisher(
            String, 'limp_and_reset_origin', 10)
        
        self.read_angels_pub = self.create_publisher( # used during new mapping
            String, 'read_angles', 10)
        
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray, 'actual_joint_angles', self.joint_angles_callback, 10)
        
        # Publishing mapping completion notification
        self.mapping_done_pub = self.create_publisher(
            String, 'system_state_request', 10)
        
        self.joints = {'left': 0, 'right': 0}
        self.mapping_data = None
        self.button_press_index = 0
        self.number_of_points = 3
        self.point_number = 0
        self.set_mapping_data()

        self.state = 'standby'

    def state_callback(self, msg):
        if msg.data == 'map':
            self.state = 'map'
            self.get_logger().info(f'mapper state set to {self.state}')
            self.start_mapping()


    def start_mapping(self):
        # Make the motors limp
        self.set_null_points_pub.publish(String(data='start'))
        self.get_logger().info('start map')
        time.sleep(25) 
        self.get_logger().info("--- move the arm to top position and press map button to record max angels. --- ") 


    def button_press_callback(self, msg):
        self.get_logger().info("Recieved button press in mapper node ") 
        self.button_press_index += 1
        self.read_angels_pub.publish(String(data='start'))

    def joint_angles_callback(self, msg):
        try:
            angles = msg.data  # Assuming this is already an array of float64

            # publish to display
            # float_array = Float64MultiArray()
            # dim = MultiArrayDimension()
            # dim.label = "joint_angles"
            # dim.size = len(angles)
            # dim.stride = len(angles)
            # float_array.layout.dim.append(dim)
            # float_array.data = angles
            # self.joint_angles_pub.publish(float_array)

            self.get_logger().info("Published joint angles for mapping.")
            lss0_angle = angles[0]
            lss1_angle = angles[1]
            
            # Append new data based on button press index
            if self.button_press_index == 1:  # Top position boundaries
                self.mapping_data['boundaries'][self.button_press_index - 1]['top_lss0'] = lss0_angle
                self.mapping_data['boundaries'][self.button_press_index - 1]['top_lss1'] = lss1_angle
                self.get_logger().info(f'Received and mapped top boundaries, angles: {angles}')
                self.get_logger().info('--- move arm to bottom position and press map button --- ')
            
            elif self.button_press_index == 2:  # Bottom position boundaries
                self.mapping_data['boundaries'][self.button_press_index - 1]['bottom_lss0'] = lss0_angle
                self.mapping_data['boundaries'][self.button_press_index - 1]['bottom_lss1'] = lss1_angle
                self.get_logger().info(f'Received and mapped bottom boundaries, angles: {angles}')
                self.get_logger().info('--- move arm to first point in path and press map button --- ')

            elif self.button_press_index >= 3:  # Path points
                self.point_number += 1
                self.mapping_data['path_points'][self.point_number - 1]['nr'] = self.point_number
                self.mapping_data['path_points'][self.point_number - 1]['lss0_angle'] = lss0_angle
                self.mapping_data['path_points'][self.point_number - 1]['lss1_angle'] = lss1_angle
                self.get_logger().info(f'Received and mapped angles for point nr {self.point_number}, angles: {angles}')
                if self.point_number < self.number_of_points:
                    self.get_logger().info(f'--- move arm to point {self.point_number + 1} in path and press map button --- ')
                else:
                    self.get_logger().info('--- all path points mapped --- ')
                    self.point_number = 0
                    self.button_press_index = 0
                    self.state = 'standby'
                    self.save_to_file()  # Save the data to file once all mapping is completed
                    self.pub_mapping_done()  # Notify that mapping is done
                    
        except Exception as e:
            self.get_logger().error(f'failed to process joint angels{e}')

        
    def pub_mapping_done(self):
        msg = String()
        msg.data = "mapping_done"
        self.get_logger().info(f'Published a flag to the system state request topic: {msg.data}')
        self.mapping_done_pub.publish(msg)

    def set_mapping_data(self):
        try:
            with open('boundary_path_and_rail_position.json', 'r') as file:
                self.mapping_data = json.load(file)
                self.get_logger().info('Successfully set the boundaries for the LSS motors, and the last rail position in the rail_position variable')

        except FileNotFoundError:
                self.get_logger().error("File boundary_path_and_rail_position.json not found.")
                return

    def save_to_file(self):
        try:
            # Write updated data to file
            with open('boundary_path_and_rail_position.json', 'w') as file:
                json.dump(self.mapping_data, file, indent=4)

            self.get_logger().info("Saved mapping data to boundary_path_and_rail_position.json.")
        except Exception as e:
            self.get_logger().error(f"Failed to save mapping data: {e}")

def main(args=None):
    rclpy.init(args=args)
    mapper = Mapper()
    rclpy.spin(mapper)
    mapper.save_to_file()
    rclpy.shutdown()

if __name__ == '__main__':
    main()