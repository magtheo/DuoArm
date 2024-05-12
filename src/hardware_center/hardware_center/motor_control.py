#!/usr/bin/env python3
# motor_control.py

## @file motor_control.py
#  @brief This script is used to control LSS servo motors during mapping and path execution.

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

## Constants for LSS Port configuration, with udev tools we have created symlinks to the physical ports, make sure to always connect usb to the same port
CST_LSS_Port = '/dev/lssMotorController'	# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

## Initialize and open a serial port for communication
initBus(CST_LSS_Port, CST_LSS_Baud)

## Create instances of LSS class for different motor IDs
lss0 = LSS(0)
lss1 = LSS(1)
lss2 = LSS(2)

## Initial motor angles and positions
lss0_act_angle = 0
lss1_act_angle = 0

lss0_act_position = 0
lss1_act_position = 0

## Gear ratio used for calculations
gear_ratio = 2


## @class motorControl
#  @brief This class encapsulates methods to control motor behavior based on received commands.
class motorControl(Node):
    def __init__(self):
        super().__init__("motor_control")

        ## Define a QoS profile for real-time updates, this is only used when sending angles to display node, which we sadly didnt manage to work properly
        self.real_time_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        ## Initialize servos, threads and subscriptions
        self.boundaries = {}
        self.load_and_set_boundaries()

        self.setup_servos_and_queues()

        self.threads = []
        self.init_servo_threads()

        self.lock = threading.Lock()
        self.current_target = None

        self.state = 'standby'

        self.total_movements = 0
        self.completed_movements = 0
        self.current_movement_nr = 0


        ## Publishers and subscribers setup
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
        
        ## Subscriber for receiving start test command
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

        self.limp_and_reset_origin_sub = self.create_subscription(
            String,
            'limp_and_reset_origin',
            self.limp_and_set_origin,
            10)
        
        self.read_angels_sub = self.create_subscription(
            String, 'read_angles', self.read_angles_callback, 10)
        
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray, 'joint_angles_array', self.iterate_and_move_servos_callback, 10)

        self.path_done_pub = self.create_publisher(
            String, 'system_state_request', 10)
        
        self.state_subscription = self.create_subscription(
            String, 'action_controller_state', self.state_callback, 10)


    ## @brief Loads and sets boundaries from a configuration file.
    def load_and_set_boundaries(self):
        try:
            with open('boundary_path_and_rail_position.json', 'r') as file:
                data = json.load(file)
                
                self.boundaries = {
                    'lss0': {'min': data['boundaries'][0]['top_lss0'], 'max': data['boundaries'][1]['bottom_lss0']},
                    'lss1': {'min': data['boundaries'][1]['bottom_lss1'], 'max': data['boundaries'][0]['top_lss1']}
                }
            self.get_logger().info(f"Boundaries loaded")
        except FileNotFoundError:
            self.get_logger().error("boundary_path_and_rail_position.json found.")
        except KeyError as e:
            self.get_logger().error(f"Error reading boundaries from file: {e}")

    ## @brief Sets up servo motors and initializes queues for handling angles.
    def setup_servos_and_queues(self):
        self.servos = {'lss0': LSS(0), 'lss1': LSS(1)}
        self.angles_queues = {key: queue.Queue() for key in self.servos}

    ## @brief Initializes threads for controlling servos.
    def init_servo_threads(self):
        for key in self.servos:
            thread = threading.Thread(target=self.servo_control_loop, args=(key,))
            thread.start()

    ## @brief Thread loop for servo control.
    #  @param servo_key The key identifier for the servo.
    def servo_control_loop(self, servo_key):
        while True:
            target_angle = self.angles_queues[servo_key].get()
            with self.lock:
                self.completed_movements += 1
            self.control_servo_speed(servo_key, target_angle)

    ## @brief Controls the speed of the servo based on target angle using a PID controller.
    #  @param servo_key Key identifier for the servo.
    #  @param target_angle Desired angle to reach.
    def control_servo_speed(self, servo_key, target_angle):
        tolerance = 1.0  # degrees within which we consider the target reached
        servo = self.servos[servo_key]
        self.get_logger().info(f" servo{servo_key} is moving to new target angle{target_angle}")

        kp = 0.2  # Proportional gain
        ki = 0.02 # Integral gain
        kd = 0.05  # Derivative gain
        integral = 0
        last_error = 0

        while True:
            with self.lock:
                current_angle = self.calc_angle(servo.getPosition())
                if current_angle is None:
                    raise ValueError("Failed to read position from servo")
                
                self.publish_joint_angles(servo_key, current_angle)  # Publish the current angles
                target_angle_minus_current_angle = target_angle - current_angle
                integral += target_angle_minus_current_angle * 0.2  # Accumulate integral (error * time)
                derivative = (target_angle_minus_current_angle - last_error) / 0.2  # Derivative (change in error / time)
                last_error = target_angle_minus_current_angle

                if np.isclose(current_angle, target_angle, atol=tolerance):
                    servo.wheelRPM(0)  # Stop the servo
                    self.get_logger().info(f"target angle {self.completed_movements}/{self.total_movements} reached for {servo_key}")
                    break

                speed = kp * target_angle_minus_current_angle + ki * integral + kd * derivative
                servo.wheelRPM(speed)

            # time.sleep(0.1)  # Adjust timing as necessary for more responsive control
            self.get_logger().info(f"Servo: {servo_key}, speed: {np.round(speed)}, difference: {np.round(target_angle_minus_current_angle)}, target angle: {target_angle}, current angle: {current_angle}")

    ## @brief Callback function for handling an array of joint angles and moving servos accordingly.
    #  @param msg Message containing joint angles.
    def iterate_and_move_servos_callback(self, msg):
        self.get_logger().info("motor_controller received array of angles")
        angles = np.array(msg.data)
        num_cycles = 3  # Number of times to repeat the path back and forth
        self.follow_path(angles, num_cycles)
        
    ## @brief Follows a predefined path of angles.
    #  @param angles Array of angles to follow.
    #  @param num_cycles Number of cycles to repeat the path.
    def follow_path(self, angles, num_cycles):
        self.total_movements = len(angles)*2*num_cycles
        for _ in range(num_cycles):
            # Forward direction
            self.get_logger().info("Moving forward")
            for i in range(0, len(angles), 2):
                self.angles_queues['lss0'].put(angles[i])
                self.angles_queues['lss1'].put(angles[i + 1])
                self.current_movement_nr += 2
                time.sleep(2)  # Wait for some time between movements

            # Wait for all movements to complete before reversing
            while self.completed_movements < self.current_movement_nr:
                time.sleep(0.5)

            self.get_logger().info("Reversing direction")
            # Reverse direction
            for i in range(len(angles) - 2, -1, -2):

                self.angles_queues['lss0'].put(angles[i])
                self.angles_queues['lss1'].put(angles[i + 1])
                self.current_movement_nr += 2
                time.sleep(2)  # Wait for some time between movements
            
            # Wait for all movements to complete before next cycle
            while self.completed_movements < self.current_movement_nr:
                time.sleep(0.1)

        # Reset movements counters after completion of all cycles
        self.completed_movements = 0
        self.total_movements = 0
        self.current_movement_nr = 0
        self.get_logger().info("All cycles completed")
        self.path_done_pub.publish(String(data="path_done"))
        self.get_logger().info(f"published done to action_controller")

    ## @brief Sets the servos to limp mode and resets the origin.
    def limp_and_set_origin(self):
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

    ## @brief Publishes the current joint angles.
    #  @param servo_key Key identifier for the servo.
    #  @param angle Current angle of the servo.
    def publish_joint_angles(self, servo_key, angle):
        data = {'servo': servo_key, 'angle': angle}
        msg = String()
        msg.data = json.dumps(data)
        self.joint_angles_publisher.publish(msg)
        self.get_logger().info(f"Published current angles for {servo_key}")
        
    
    ## @brief Calculates the position used by LSS servos based on an angle. This position is angle*10 (ex: 14,5 deg = 145 position)
    #  @param angle The angle to calculate the position for.
    #  @return The calculated position.
    def calc_position(self, angle):
        return angle*10
    
    ## @brief Calculates the angle based on a position.
    #  @param position The position to calculate the angle for.
    #  @return The calculated angle or None if an error occurs.
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

    ## @brief Callback for reading angles from servos.
    def read_angles_callback(self):
        # Fetch current positions from servos
        lss0_act_position = float(lss0.getPosition())
        lss1_act_position = float(lss1.getPosition())

        # Convert positions back to angles
        lss0_act_angle = self.calc_angle(lss0_act_position)
        lss1_act_angle = self.calc_angle(lss1_act_position)
        actual_joint_angles = [lss0_act_angle, lss1_act_angle]
        
        transmission_msg = Float64MultiArray()
        transmission_msg.data = actual_joint_angles
        self.actual_joint_angles_publisher.publish(transmission_msg) # mapping
        self.get_logger().info(f"Published angle readings: LSS0:{lss0_act_angle}| LSS1:{lss1_act_angle}")

    ## @brief Manual callback used during testing to set servos to limp mode and read positions.
    def manualy_callback(self): # used during testing
        self.get_logger().info( 'test ')
        lss0.limp()
        lss1.limp()

        for i in range(2):
            self.get_logger().info(f'current loop: {i}')

            time.sleep(2)

            self.get_logger().info(f'lss0, right: {self.calc_angle(lss0.getPosition())}')
            self.get_logger().info(f'lss1, left: {self.calc_angle(lss1.getPosition())}')

    ## @brief Callback for starting the read angles test.
    def start_read_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting to read motor angles')
            self.manualy_callback()    


    ## @brief Checks the current state and stops movements if necessary. We did not get this to work as the local state could not be updated whilst the servos where moving.
    def check_state_and_stop(self):            
        self.get_logger().info('enterd check state and stop predefined path')
        """Check the current state and stop movement if it's changed to standby."""
        if self.state == 'standby':
            # Stop the movement of servos
            for servo_key in self.servos:
                self.get_logger().info('stop path, for loop iteration')
                self.angles_queues[servo_key].queue.clear()  # Clear the angles queue
                self.servos[servo_key].wheelRPM(0)  # Stop the servo
            self.completed_movements = 0
            self.total_movements = 0

    ## @brief Callback for updating the state based on received message.
    #  @param msg Message containing the new state.
    def state_callback(self, msg):
        self.state = msg.data
        self.get_logger().info(f'motor controller state updated: {self.state}')


## @brief Main function to initialize the ROS node.
def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()