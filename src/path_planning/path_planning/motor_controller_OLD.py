#!/usr/bin/env python3
# motor_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import time
import json
import threading
from .lss import *
import queue
import math

CST_LSS_Port = '/dev/lssMotorController'	# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

# Create and open a serial port
initBus(CST_LSS_Port, CST_LSS_Baud)

lss0 = LSS(0)
lss1 = LSS(1)
lss2 = LSS(2)

lss0_act_angle = 0
lss1_act_angle = 0

lss0_act_position = 0
lss1_act_position = 0

gear_ratio = 2

received_joint_angles = np.zeros(2)
transmission_msg = Float64MultiArray()



class motorControl(Node):
    def __init__(self):
        super().__init__("motor_control")

        # Define a QoS profile for real-time updates
        self.real_time_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Initialize other attributes and subscriptions...
        self.boundaries = {}
        self.load_and_set_boundaries()

        self.setup_servos_and_queues()

        self.threads = []
        self.init_servo_threads()

        self.lock = threading.Lock()
        self.current_target = None

       # Initialize boundary values
        self.bottom_lss0 = None
        self.top_lss0 = None
        self.bottom_lss1 = None
        self.top_lss1 = None

        self.state = 'standby'

        self.total_movements = 0
        self.completed_movements = 0

        self.calc_joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'calculated_joint_angles',
            self.calc_joint_angles_callback,
            10
        )

        
        self.actual_joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 
            'actual_joint_angles', # used during mapping and motor control
            10
        )

        self.joint_angles_publisher = self.create_publisher(
            String,
            'joint_angles',  # Topic to publish the joint angles to display node
            qos_profile=self.real_time_qos
        )
        
        # Subscriber for receiving start test command
        self.start_read_sub = self.create_subscription(
            String,
            'start_read',
            self.start_read_callback,
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
        
       

    def limp_and_set_origin(self, msg):
        """
        Makes the servos go limp, waits for X seconds, and then sets a new origin offset.

        :param new_origin_offset: The new origin offset to be set for the servos.
        """
        self.get_logger().info('Making arm limp...')
        # Make the arm limp
        lss0.limp()
        lss1.limp()
        lss2.limp()

        lss0.setGyre(-1, LSS_SetConfig)
        lss1.setGyre(-1, LSS_SetConfig)

        self.get_logger().info(f'Motors are now limp, move arm for desired position, null points will be set in 25 sec')
        time.sleep(25)

        # Set new origin offset
        lss0.setOriginOffset(0, LSS_SetConfig)
        lss1.setOriginOffset(0, LSS_SetConfig)
        lss2.setOriginOffset(0, LSS_SetConfig)

        self.get_logger().info(f'actual angles for lss0: {self.calc_angle(lss0.getPosition())}')
        self.get_logger().info(f'actual angles for lss1: {self.calc_angle(lss1.getPosition())}')
        self.get_logger().info(f'actual angles for lss2, rail: {self.calc_angle(lss2.getPosition())}')

        new_origin_offset0 = int(lss0.getPosition()) 
        new_origin_offset1 = int(lss1.getPosition())
        self.get_logger().info('Setting new origin offset...')
        lss0.setOriginOffset(new_origin_offset0, LSS_SetConfig)
        lss1.setOriginOffset(new_origin_offset1, LSS_SetConfig)

        self.get_logger().info(f'New origin offset set to {new_origin_offset0, new_origin_offset1}.')

        self.get_logger().info(f'actual angles after new null point for lss0: {self.calc_angle(lss0.getPosition())}')
        self.get_logger().info(f'actual angles after new null point for lss1: {self.calc_angle(lss1.getPosition())}')
        if self.calc_angle(lss1.getPosition()) == -180.0:
            lss1.setOriginOffset(-1800, LSS_SetConfig)

    def set_target_angles_callback(self, request, response):
        target_angles = request.target_angles
        # Code to move the motors to target_angles
        self.move_servos_mapping(target_angles)
        actual_angles = self._callback()
        response.actual_angles = actual_angles
        response.success = True  # Change based on actual movement success
        response.message = 'Moved successfully'  # Change based on actual movement success
        return response

    def publish_joint_angles(self, servo_key, angle):
        data = {'servo': servo_key, 'angle': angle}
        msg = String()
        msg.data = json.dumps(data)
        self.joint_angles_publisher.publish(msg)
        self.get_logger().info(f"Published current angles for {servo_key}")
        
    def calc_joint_angles_callback(self, msg):
        received_joint_angles = msg.data
        self.get_logger().info(f'Received calculated joint angles: {received_joint_angles}')
        self.move_servos_mapping(received_joint_angles)
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
            joint_angle = (numeric_position / 10) #* gear_ratio
            return joint_angle
        except ValueError as e:
            self.get_logger().error(f"Error converting position to float: {e}")
            return None  # Or handle the error in a way that suits your application


    def move_servos_mapping(self, received_joint_angles):
        deg0 = received_joint_angles[1] #/ gear_ratio
        deg1 = received_joint_angles[0] #/ gear_ratio

        # move servo if within limits
        lss0_position = self.calc_position(deg0)
        lss1_position = self.calc_position(deg1)
        lss0.move(lss0_position)
        lss1.move(lss1_position)
        self.get_logger().info(f"lss0 tried moving to angle{deg0}")
        self.get_logger().info(f"lss1 tried moving to angle{deg1}")
        self.get_logger().info(f"lss0 tried moving to pos{lss0_position}")
        self.get_logger().info(f"lss1 tried moving to pos{lss1_position}")


    def read_and_pub_servo_angles(self): # USED during old mapping sequence
        # Fetch current positions from servos, convert to angles, and publish
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())
        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.actual_joint_angles_publisher.publish(transmission_msg)
        self.get_logger().info(f"Published angle readings: LSS0:{lss0_act_angle}| LSS1:{lss1_act_angle}")

    def ref_point_read_and_pub_servo_angles(self):
        # Fetch current positions from servos, convert to angles, and publish
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())
        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.manual_readings_pub.publish(transmission_msg)
        self.get_logger().info(f"Published angle readings: LSS0:{lss0_act_angle}| LSS1:{lss1_act_angle}")


    def manualy_callback(self): # used to manualy read angles during testin
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
            self.manualy_callback()    

    def read_ref_point_callback(self, msg):
        if msg.data == "start":
            lss0.limp()
            lss1.limp()
            self.ref_point_read_and_pub_servo_angles()


    def state_callback(self, msg):
        self.state = msg.data
        self.get_logger().info(f'motor controller state updated: {self.state}')



def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()