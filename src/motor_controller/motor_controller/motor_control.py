#!/usr/bin/env python3
# motor_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

# from controll_center.srv import MoveServos


# from auto_mapper import MIN_THETA1_LEFT, MAX_THETA1_LEFT, MIN_THETA1_RIGHT, MAX_THETA1_RIGHT
# fix import

from .lss import *

CST_LSS_Port = "/dev/ttyUSB1"		# For Linux/Unix platforms
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

        self.manual_readings_pub = self.create_publisher(
            Float64MultiArray, 
            'manual_angle_readings',
            10
        )
        # Subscriber for receiving start test command
        self.start_read_sub = self.create_subscription(
            String,
            'start_ref_read',
            self.read_ref_point_callback,
            10
        )

        self.limp_and_reset_origin_sub = self.create_subscription(
            String,
            'limp_and_reset_origin',
            self.limp_and_set_origin,
            10)
        
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles_array',
            self.iterate_and_move_servos_callback,
            10
        )

        self.path_done_pub = self.create_publisher(
            String,
            'path_done',
            10
        )
        
        # self.srv = self.create_service(MoveServos, 'set_target_angles', self.set_target_angles_callback)

        # Define angle limits
        self.MIN_THETA1_LEFT = np.radians(-140)
        self.MAX_THETA1_LEFT = np.radians(15)
        self.MIN_THETA1_RIGHT = np.radians(-50)
        self.MAX_THETA1_RIGHT = np.radians(140)
        
    def set_target_angles_callback(self, request, response):
        target_angles = request.target_angles
        # Code to move the motors to target_angles
        self.move_servos_mapping(target_angles)
        actual_angles = self.read_angles()
        response.actual_angles = actual_angles
        response.success = True  # Change based on actual movement success
        response.message = 'Moved successfully'  # Change based on actual movement success
        return response


    def calc_joint_angles_callback(self, msg):
        received_joint_angles = msg.data
        self.get_logger().info(f'Received calculated joint angles: {np.rad2deg(received_joint_angles)}')
        self.move_servos_mapping(received_joint_angles)
        # Optionally, introduce a delay or use a more sophisticated mechanism
        # to ensure servos have reached their positions before reading
        time.sleep(1)  # TODO adjust delay
        self.read_and_pub_servo_angles()


    
    #def send_actual_joint_angles(self):
    def calc_position(self, angle):
        return angle*10
    
    def calc_angle(self, position):
        if position is None:
            self.get_logger().error("Failed to read position from servo")
            return None  # Return None or a default value
        try:
            numeric_position = float(position)  # Safely convert position to float
            return numeric_position / 10
        except ValueError as e:
            self.get_logger().error(f"Error converting position to float: {e}")
            return None  # Or handle the error in a way that suits your application


    def move_servos_mapping(self, received_joint_angles):
        # self.get_logger().info(f'Received calculated joint angles: {np.rad2deg(received_angle_left_rad), np.rad2deg(received_angle_right_rad)}')
        deg0 = np.rad2deg(received_joint_angles[1])
        deg1 = np.rad2deg(received_joint_angles[0])

        # move servo if within limits
        lss0_position = self.calc_position(deg0)
        lss1_position = self.calc_position(deg1)
        lss0.move(lss0_position)
        lss1.move(lss1_position)
        self.get_logger().info(f"lss0 tried moving to angle{deg0}")
        self.get_logger().info(f"lss1 tried moving to angle{deg1}")
        self.get_logger().info(f"lss0 tried moving to pos{lss0_position}")
        self.get_logger().info(f"lss1 tried moving to pos{lss1_position}")

    def read_angles(self):
        # Fetch current positions from servos
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())

        # Convert positions back to angles
        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        return actual_joint_angles


    def manualy_read_and_pub_servo_angles(self):
        # Fetch current positions from servos, convert to angles, and publish
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())
        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.manual_readings_pub.publish(transmission_msg)
        self.get_logger().info(f"Published manual angle readings: LSS0:{lss0_act_angle}| LSS1:{lss1_act_angle}")


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

    def iterate_and_move_servos_callback(self, msg):
        joint_angles = np.array(msg.data)
        num_servos = 2  # Number of servos controlled

        if len(joint_angles) % num_servos != 0:
            self.get_logger().error("Received joint angles array is not a multiple of the number of servos.")
            return

        # Loop to continuously move back and forth
        for i in range(4):
            # Forward iteration
            for i in range(0, len(joint_angles), num_servos):
                angle_pair = joint_angles[i:i+num_servos]
                self.move_servos_to_angles(angle_pair)
                time.sleep(0.5)  # Delay to allow for servo motion before the next command

            # Reverse iteration
            for i in range(len(joint_angles) - num_servos, -1, -num_servos):
                angle_pair = joint_angles[i:i+num_servos]
                self.move_servos_to_angles(angle_pair)
                time.sleep(0.5)  # Delay to allow for servo motion before the next command

        # Notify completion of the return path
        self.path_done_pub.publish(String(data="done"))

    def move_servos_to_angles(self, angles):
        """ Moves the servos to the specified angles received in a pair [lss0_angle, lss1_angle] """
        if len(angles) != 2:
            self.get_logger().error("Angle pair does not contain exactly two elements.")
            return

        lss0_angle_deg = np.rad2deg(angles[0])
        lss1_angle_deg = np.rad2deg(angles[1])

        lss0_position = self.calc_position(lss0_angle_deg)
        lss1_position = self.calc_position(lss1_angle_deg)

        if self.MIN_THETA1_LEFT <= angles[0] <= self.MAX_THETA1_LEFT and \
           self.MIN_THETA1_RIGHT <= angles[1] <= self.MAX_THETA1_RIGHT:
            lss0.move(lss0_position)
            lss1.move(lss1_position)
            self.get_logger().info(f"Moved LSS0 to {lss0_angle_deg} degrees, LSS1 to {lss1_angle_deg} degrees")
        else:
            self.get_logger().warn("Received angles are out of bounds")
            self.out_of_bounds_publisher.publish(String(data="Angles out of bounds"))

    def limp_and_set_origin(self, msg):
        """
        Makes the servos go limp, waits for X seconds, and then sets a new origin offset.

        :param new_origin_offset: The new origin offset to be set for the servos.
        """
        self.get_logger().info('Making arm limp...')
        # Make the arm limp
        lss0.limp()
        lss1.limp()

        lss0.setGyre(-1, LSS_SetConfig)
        lss1.setGyre(-1, LSS_SetConfig)

        # Set new origin offset
        lss0.setOriginOffset(0, LSS_SetConfig)
        lss1.setOriginOffset(1800, LSS_SetConfig)

        self.get_logger().info(f'actual angles for lss0: {self.calc_angle(lss0.getPosition())}')
        self.get_logger().info(f'actual angles for lss1: {self.calc_angle(lss1.getPosition())}')

        new_origin_offset0 = int(lss0.getPosition()) 
        new_origin_offset1 = int(lss1.getPosition())
        self.get_logger().info('Setting new origin offset...')
        lss0.setOriginOffset(new_origin_offset0, LSS_SetConfig)
        lss1.setOriginOffset(new_origin_offset1, LSS_SetConfig)

        self.get_logger().info(f'New origin offset set to {new_origin_offset0, new_origin_offset1}.')

        self.get_logger().info(f'actual angles after new null point for lss0: {self.calc_angle(lss0.getPosition())}')
        self.get_logger().info(f'actual angles after new null point for lss1: {self.calc_angle(lss1.getPosition())}')
        time.sleep(3)
        self.get_logger().info(f'actual angles after new null point for lss0: {self.calc_angle(lss0.getPosition())}')
        self.get_logger().info(f'actual angles after new null point for lss1: {self.calc_angle(lss1.getPosition())}')


    def start_test_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting test of motor movement')
            self.test_motors()

    def manualy_read_angles(self):
        self.get_logger().info( 'test ')
        lss0.limp()
        lss1.limp()

        for i in range(2):
            self.get_logger().info(f'current loop: {i}')

            time.sleep(2)

            self.get_logger().info(f'lss0, right: {self.calc_angle(lss0.getPosition())}')
            self.get_logger().info(f'lss1, left: {self.calc_angle(lss1.getPosition())}')

    def start_read_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting to read motor angles')
            self.manualy_read_angles()    

    def read_ref_point_callback(self, msg):
        if msg.data == "start":
            lss0.limp()
            lss1.limp()
            self.manualy_read_and_pub_servo_angles()


def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()


