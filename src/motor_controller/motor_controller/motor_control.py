#!/usr/bin/env python3
# motor_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

from .lss import *

CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

# Create and open a serial port
initBus(CST_LSS_Port, CST_LSS_Baud)

lss0 = LSS(0)
lss1 = LSS(1)

lss0_act_angle = 0
lss1_act_angle = 0

lss0_act_position = 0
lss1_act_position = 0

received_joint_angles = np.zeros(2)
active_joint_angles = np.zeros(2)
transmission_msg = Float64MultiArray()

class motorControl(Node):
    def __init__(self):
        super().__init__("motor_control")

        self.calc_joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'calculated_joint_angles',
            self.calc_joint_angles_callback,
            10
        )
        self.active_joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 
            'actual_joint_angles',
            10
        )

        # Subscriber for receiving start test command
        self.start_test_sub = self.create_subscription(
            String,
            'start_test',
            self.start_test_callback,
            10)
        
        # Subscriber for receiving start test command
        self.start_read_sub = self.create_subscription(
            String,
            'start_read',
            self.start_read_callback,
            10)
        
        # Define angle limits
        self.MIN_THETA1_LEFT = np.radians(-13)
        self.MAX_THETA1_LEFT = np.radians(90)
        self.MIN_THETA1_RIGHT = np.radians(-90)
        self.MAX_THETA1_RIGHT = np.radians(13)
        
    def calc_joint_angles_callback(self, msg):
        received_joint_angles = msg.data
        self.get_logger().info(f'Received calculated joint angles: {received_joint_angles}')
        self.move_servos_mapping(received_joint_angles)
        # Optionally, introduce a delay or use a more sophisticated mechanism
        # to ensure servos have reached their positions before reading
        time.sleep(2)  # TODO Example delay, adjust based on your system's characteristics
        self.read_and_publish_servo_positions()

    
    #def send_actual_joint_angles(self):
    def calc_position(self, angle):
        return angle*10
    
    def calc_angle(self, position):
        return position/10

    def move_servos_mapping(self, received_joint_angles):
        # Convert received angles from degrees to radians for comparison
        received_angle_left_rad = np.radians(received_joint_angles[0])
        received_angle_right_rad = np.radians(received_joint_angles[1])

        # Check and move left servo if within limits
        if self.MIN_THETA1_LEFT <= received_angle_left_rad <= self.MAX_THETA1_LEFT:
            lss0_position = self.calc_position(received_joint_angles[0])
            lss0.move(lss0_position)
        else:
            self.get_logger().warn(f"Left motor angle {received_joint_angles[0]} out of bounds.")

        # Check and move right servo if within limits
        if self.MIN_THETA1_RIGHT <= received_angle_right_rad <= self.MAX_THETA1_RIGHT:
            lss1_position = self.calc_position(received_joint_angles[1])
            lss1.move(lss1_position)
        else:
            self.get_logger().warn(f"Right motor angle {received_joint_angles[1]} out of bounds.")

    def read_and_publish_servo_positions(self):
        # Fetch current positions from servos
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())

        # Convert positions back to angles
        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)

        # Prepare and publish the actual angles
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.active_joint_angles_publisher.publish(transmission_msg)
        self.get_logger().info(f'Published actual joint angles: {actual_joint_angles}')

    
    def test_motors(self):
        self.get_logger().info( 'test ')

        lss1.move(-900)
        time.sleep(2)
        self.get_logger().info(lss0.getPosition())
        self.get_logger().info(lss1.getPosition())

        # lss0.move(0)
        lss1.move(0)
        time.sleep(2)
        self.get_logger().info(lss0.getPosition())
        self.get_logger().info(lss1.getPosition())  
        
        for i in range(4):
            self.get_logger().info(f'current loop: {i}')

            # lss0.move(0)
            lss1.move(500)
            time.sleep(2)
            self.get_logger().info(lss0.getPosition())
            self.get_logger().info(lss1.getPosition())

            lss1.move(0)
            time.sleep(2)
            self.get_logger().info(lss0.getPosition())
            self.get_logger().info(lss1.getPosition())


    def start_test_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting test of motor movement')
            self.test_motors()

    def manualy_read_angles(self):
        self.get_logger().info( 'test ')

        for i in range(4):
            self.get_logger().info(f'current loop: {i}')

            time.sleep(2)

            self.get_logger().info(f'lss0, left: {lss0.getPosition()}')
            self.get_logger().info(f'lss1, right: {lss1.getPosition()}')

    def start_read_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting to read motor angles')
            self.manualy_read_angles()    


def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()


