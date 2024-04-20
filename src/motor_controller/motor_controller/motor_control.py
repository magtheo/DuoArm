#!/usr/bin/env python3
# motor_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

# from auto_mapper import MIN_THETA1_LEFT, MAX_THETA1_LEFT, MIN_THETA1_RIGHT, MAX_THETA1_RIGHT
# fix import

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
            10
        )
        
        self.out_of_bounds_publisher = self.create_publisher(
            String, 
            'out_of_bounds',
            10
        )


        # Subscriber for receiving start test command
        self.set_origin_sub = self.create_subscription(
            String,
            'set_origin',
            self.set_origin_callback,
            10
        )

        self.manual_readings_pub = self.create_publisher(
            Float64MultiArray, 
            'manual_angle_readings',
            10
        )
        # Subscriber for receiving start test command
        self.start_read_sub = self.create_subscription(
            String,
            'start_read',
            self.read_ref_point_callback,
            10
        )

        
        

        # Define angle limits
        self.MIN_THETA1_LEFT = np.radians(-140)
        self.MAX_THETA1_LEFT = np.radians(15)
        self.MIN_THETA1_RIGHT = np.radians(-50)
        self.MAX_THETA1_RIGHT = np.radians(140)
        
    def calc_joint_angles_callback(self, msg):
        received_joint_angles = msg.data
        self.get_logger().info(f'Received calculated joint angles: {np.rad2deg(received_joint_angles)}')
        self.move_servos_mapping(received_joint_angles)
        # Optionally, introduce a delay or use a more sophisticated mechanism
        # to ensure servos have reached their positions before reading
        time.sleep(1)  # TODO adjust delay
        self.read_and_pub_servo_angels()


    
    #def send_actual_joint_angles(self):
    def calc_position(self, angle):
        return angle*10
    
    def calc_angle(self, position):
        return position/10

    def move_servos_mapping(self, received_joint_angles):
        # self.get_logger().info(f'Received calculated joint angles: {np.rad2deg(received_angle_left_rad), np.rad2deg(received_angle_right_rad)}')
        deg0 = np.rad2deg(received_joint_angles[0])
        deg1 = np.rad2deg(received_joint_angles[1])

        # move servo if within limits
        lss0_position = self.calc_position(deg0)
        lss1_position = self.calc_position(deg1)
        lss0.move(lss0_position)
        lss1.move(lss1_position)
        self.get_logger().info(f"lss0 tried to moved to angle{deg0}")
        self.get_logger().info(f"lss1 tried to moved to angle{deg1}")
        self.get_logger().info(f"lss0 tried to moved to pos{lss0_position}")
        self.get_logger().info(f"lss1 tried to moved to pos{lss1_position}")


       
    def read_and_pub_servo_angles(self):
        # Fetch current positions from servos
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())

        # Convert positions back to angles
        lss0_act_angle = np.deg2rad(self.calc_angle(lss0_act_position))
        lss1_act_angle = np.deg2rad(self.calc_angle(lss1_act_position))

        # Prepare and publish the actual angles
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.active_joint_angles_publisher.publish(transmission_msg)
        self.get_logger().info(f'Published actual joint angles: {np.rad2deg(actual_joint_angles)}')


    def manualy_read_and_pub_servo_angles(self):
        # Fetch current positions from servos, convert to angles, and publish
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())
        lss0_act_angle = np.deg2rad(self.calc_angle(lss0_act_position))
        lss1_act_angle = np.deg2rad(self.calc_angle(lss1_act_position))
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.manual_readings_pub.publish(transmission_msg)
        self.get_logger().info(f"Published manual angle readings: {actual_joint_angles}")


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

    def read_ref_point_callback(self, msg):
        if msg.data == "start":
            self.manualy_read_and_pub_servo_angles()


def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()


