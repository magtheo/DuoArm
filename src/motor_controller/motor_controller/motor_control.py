#!/usr/bin/env python3
# motor_control.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np
import time
import json
import threading
import json
import threading
from .lss import *
import queue
import math
import queue
import math

CST_LSS_Port = "/dev/ttyUSB0"		# For Linux/Unix platforms
#CST_LSS_Port = "COM230"				# For windows platforms
CST_LSS_Baud = LSS_DefaultBaud

# Create and open a serial port
initBus(CST_LSS_Port, CST_LSS_Baud)

lss0 = LSS(0)
lss1 = LSS(1)
lss2 = LSS(2)
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

        self.state = 'standby'

       # Initialize boundary values
        self.bottom_lss0 = None
        self.top_lss0 = None
        self.bottom_lss1 = None
        self.top_lss1 = None

        self.total_movements = 0
        self.completed_movements = 0

        self.state_subscription = self.create_subscription(
            String, 'action_controller_state', self.state_callback, 10)

        self.calc_joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'calculated_joint_angles',
            self.calc_joint_angles_callback,
            10
        )


        self.joint_angles_publisher = self.create_publisher(
            String,
            'joint_angles',  # Topic to publish the joint angles to display node
            qos_profile=self.real_time_qos
        )


        self.joint_angles_publisher = self.create_publisher(
            String,
            'joint_angles',  # Topic to publish the joint angles to display node
            qos_profile=self.real_time_qos
        )

        # Subscriber for receiving start test command
        self.start_test_sub = self.create_subscription( # starting test form command
            String,
            'start_test',
            self.start_test_callback,
            10)
        
        # Subscriber for receiving start test command
        self.start_read_sub = self.create_subscription( # manualy reading angles with command
            String,
            'start_read',
            self.start_read_callback,
            10
        )
        
        self.out_of_bounds_publisher = self.create_publisher( # out of bounds publisher from old mapping sequance
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

        self.limp_and_reset_origin_sub = self.create_subscription(# make limp and sett origin at the start of mapping sequence
            String, 'limp_and_reset_origin', self.limp_and_set_origin, 10)
        
        self.actual_joint_angles_publisher = self.create_publisher(# used during mapping and motor control
            Float64MultiArray, 'actual_joint_angles', 10)

        self.read_angels_sub = self.create_subscription( # used during mapping
            String, 'read_angles', self.read_angles_callback, 10)
        
        self.joint_angles_subscription = self.create_subscription( # array with angles from path node
            Float64MultiArray, 'joint_angles_array', self.iterate_and_move_servos_callback, 10)

        self.path_done_pub = self.create_publisher(
            String, 'path_done', 10)

  
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

    def setup_servos_and_queues(self):
        self.servos = {'lss0': LSS(0), 'lss1': LSS(1)}
        self.angles_queues = {key: queue.Queue() for key in self.servos}

    def init_servo_threads(self):
        for key in self.servos:
            thread = threading.Thread(target=self.servo_control_loop, args=(key,))
            thread.start()

    def servo_control_loop(self, servo_key):
        while True:
            target_angle = self.angles_queues[servo_key].get()
            self.control_servo_speed(servo_key, target_angle)
            with self.lock:
                self.completed_movements += 1
                if self.completed_movements == self.total_movements:
                    self.path_done_pub.publish(String(data="done"))
                    self.get_logger().info(f"published done to action_controller")





    def control_servo_speed(self, servo_key, target_angle):
        tolerance = 1.0  # degrees within which we consider the target reached
        servo = self.servos[servo_key]
        self.get_logger().info(f"Servo:{servo}, servo_key{servo_key}")

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
            self.get_logger().info(f"Servo: {servo}, speed: {speed}, difference: {target_angle_minus_current_angle}, target angle: {target_angle}, current angle: {current_angle}")

    def iterate_and_move_servos_callback(self, msg):
        self.get_logger().info("motor_controller received array of angles")
        angles = np.array(msg.data)
        num_cycles = 3  # Number of times to repeat the path back and forth
        
        for _ in range(num_cycles):
            # Forward direction
            self.get_logger().info("Moving forward")
            for i in range(0, len(angles), 2):
                # Check the state before moving each pair of angles
                self.check_state_and_stop()
                if self.state == 'standby':
                    return  # Exit the method if state is standby
                self.angles_queues['lss0'].put(angles[i])
                self.angles_queues['lss1'].put(angles[i + 1])
                self.total_movements += 1
                time.sleep(2)  # Wait for some time between movements

            # Wait for all movements to complete before reversing
            while self.completed_movements < self.total_movements:
                time.sleep(0.5)

            self.get_logger().info("Reversing direction")
            # Reverse direction
            for i in range(len(angles) - 2, -1, -2):
                # Check the state before moving each pair of angles
                self.check_state_and_stop()
                if self.state == 'standby':
                    return  # Exit the method if state is standby
                self.angles_queues['lss0'].put(angles[i])
                self.angles_queues['lss1'].put(angles[i + 1])
                self.total_movements += 1
                time.sleep(2)  # Wait for some time between movements
            
            # Wait for all movements to complete before next cycle
            while self.completed_movements < self.total_movements:
                time.sleep(0.1)
                self.check_state_and_stop()  # Check state continuously

        # Reset movements counters after completion of all cycles
        self.completed_movements = 0
        self.total_movements = 0
        self.get_logger().info("All cycles completed")

    
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

        self.get_logger().info(f'Motors are now limp, move arm for desired position, null points will be set in 5 sec')
        time.sleep(5)

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

    # def notify_completion(self):
    #     # Handle the completion of all movements
    #     self.path_done_pub.publish(String(data="done"))

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
        
        self.out_of_bounds_publisher = self.create_publisher( # out of bounds publisher from old mapping sequance
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
        
        self.read_angels_sub = self.create_subscription( # used during mapping
            String, 'read_angles', self.read_angles_callback, 10)
        
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray, 'joint_angles_array', self.iterate_and_move_servos_callback,
            10
        )

        self.path_done_pub = self.create_publisher(
            String,
            'path_done',
            10
        )

        
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
            joint_angle = (numeric_position / 10) #* gear_ratio
            joint_angle = (numeric_position / 10) #* gear_ratio
            return joint_angle
        except ValueError as e:
            self.get_logger().error(f"Error converting position to float: {e}")
            return None  # Or handle the error in a way that suits your application


    def move_servos_mapping(self, received_joint_angles):
        deg0 = received_joint_angles[1] #/ gear_ratio
        deg1 = received_joint_angles[0] #/ gear_ratio
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

    def read_angles_callback(self, msg):
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
        self.actual_joint_angles_publisher.publish(transmission_msg) # mapping
        self.get_logger().info(f"Published angle readings: LSS0:{lss0_act_angle}| LSS1:{lss1_act_angle}")


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

# INACTIVE
    # def test_motors(self):
    #     self.get_logger().info( 'test ')
# INACTIVE
    # def test_motors(self):
    #     self.get_logger().info( 'test ')

    #     lss1.move(-900) # GPT; but this works
    #     time.sleep(2)
    #     self.get_logger().info(lss0.getPosition())
    #     self.get_logger().info(lss1.getPosition())
    #     lss1.move(-900) # GPT; but this works
    #     time.sleep(2)
    #     self.get_logger().info(lss0.getPosition())
    #     self.get_logger().info(lss1.getPosition())

    #     # lss0.move(0)
    #     lss1.move(0)
    #     time.sleep(2)
    #     self.get_logger().info(lss0.getPosition())
    #     self.get_logger().info(lss1.getPosition())  
    #     # lss0.move(0)
    #     lss1.move(0)
    #     time.sleep(2)
    #     self.get_logger().info(lss0.getPosition())
    #     self.get_logger().info(lss1.getPosition())  
        
    #     for i in range(4):
    #         self.get_logger().info(f'current loop: {i}')
    #     for i in range(4):
    #         self.get_logger().info(f'current loop: {i}')

    #         # lss0.move(0)
    #         lss1.move(500)
    #         time.sleep(2)
    #         self.get_logger().info(lss0.getPosition())
    #         self.get_logger().info(lss1.getPosition())
    #         # lss0.move(0)
    #         lss1.move(500)
    #         time.sleep(2)
    #         self.get_logger().info(lss0.getPosition())
    #         self.get_logger().info(lss1.getPosition())

    #         lss1.move(0)
    #         time.sleep(2)
    #         self.get_logger().info(lss0.getPosition())
    #         self.get_logger().info(lss1.getPosition())
    #         lss1.move(0)
    #         time.sleep(2)
    #         self.get_logger().info(lss0.getPosition())
    #         self.get_logger().info(lss1.getPosition())

    # def iterate_and_move_servos_callbackV1(self, msg):
    #     joint_angles = np.array(msg.data)
    #     num_servos = 2  # Number of servos controlled
    # def iterate_and_move_servos_callbackV1(self, msg):
    #     joint_angles = np.array(msg.data)
    #     num_servos = 2  # Number of servos controlled

    #     if len(joint_angles) % num_servos != 0:
    #         self.get_logger().error("Received joint angles array is not a multiple of the number of servos.")
    #         return
        
    #     self.load_and_set_boundaries() # load boundrys 

    #     # Loop to continuously move back and forth
    #     for i in range(2):
    #         # Forward iteration
    #         for i in range(0, len(joint_angles), num_servos):
    #             angle_pair = joint_angles[i:i+num_servos]
    #             self.move_servos_to_anglesTHREAD(angle_pair)
    #             time.sleep(0.5)  # Delay to allow for servo motion before the next command
    #     if len(joint_angles) % num_servos != 0:
    #         self.get_logger().error("Received joint angles array is not a multiple of the number of servos.")
    #         return
        
    #     self.load_and_set_boundaries() # load boundrys 

    #     # Loop to continuously move back and forth
    #     for i in range(2):
    #         # Forward iteration
    #         for i in range(0, len(joint_angles), num_servos):
    #             angle_pair = joint_angles[i:i+num_servos]
    #             self.move_servos_to_anglesTHREAD(angle_pair)
    #             time.sleep(0.5)  # Delay to allow for servo motion before the next command

    #         # Reverse iteration
    #         for i in range(len(joint_angles) - num_servos, -1, -num_servos):
    #             angle_pair = joint_angles[i:i+num_servos]
    #             self.move_servos_to_anglesTHREAD(angle_pair)
    #             time.sleep(0.5)  # Delay to allow for servo motion before the next command
    #         # Reverse iteration
    #         for i in range(len(joint_angles) - num_servos, -1, -num_servos):
    #             angle_pair = joint_angles[i:i+num_servos]
    #             self.move_servos_to_anglesTHREAD(angle_pair)
    #             time.sleep(0.5)  # Delay to allow for servo motion before the next command

    #     # Notify completion of the return path
    #     self.path_done_pub.publish(String(data="done"))
    #     # Notify completion of the return path
    #     self.path_done_pub.publish(String(data="done"))

    # def move_servos_to_anglesTHREAD_V1(self, angles):
    #     """ Moves the servos to the specified angles received in a pair [lss0_angle, lss1_angle]. """
    #     if len(angles) != 2:
    #         self.get_logger().error("Angle pair does not contain exactly two elements.")
    #         return
    # def move_servos_to_anglesTHREAD_V1(self, angles):
    #     """ Moves the servos to the specified angles received in a pair [lss0_angle, lss1_angle]. """
    #     if len(angles) != 2:
    #         self.get_logger().error("Angle pair does not contain exactly two elements.")
    #         return

    #     lss0_angle_deg, lss1_angle_deg = angles
    #     target_lss0_position = self.calc_position(lss0_angle_deg)
    #     target_lss1_position = self.calc_position(lss1_angle_deg)
    #     self.get_logger().info(f"Target positions: LSS0 {target_lss0_position}, LSS1 {target_lss1_position}")

    #     # Creating and starting threads for each servo movement:
    #     thread0 = threading.Thread(target=self.move_servo, args=(lss0, target_lss0_position))
    #     thread1 = threading.Thread(target=self.move_servo, args=(lss1, target_lss1_position))

    #     thread0.start()
    #     thread1.start()
        
    #     thread0.join()
    #     thread1.join()

    # def move_servo(self, servo, angle):
    #     if servo == lss0:
    #         lss0.move(angle)
    #         self.get_logger().info(f"Tried moving LSS0 to angle: {angle}")
    #     elif servo == lss1:
    #         lss1.move(angle) # GPT, why wont this work
    #         self.get_logger().info(f"Tried moving LSS1 to angle: {angle}")

    # def move_servos_to_angles(self, angles):
    #     """ Moves the servos to the specified angles received in a pair [lss0_angle, lss1_angle] """
    #     if len(angles) != 2:
    #         self.get_logger().error("Angle pair does not contain exactly two elements.")
    #         return

    #     lss0_angle_deg = angles[0]
    #     lss1_angle_deg = angles[1]

    #     lss0_position = self.calc_position(lss0_angle_deg)
    #     lss1_position = self.calc_position(lss1_angle_deg)

    #     self.get_logger().info(f"LSS0, bottom boundary: {self.bottom_lss0}, move angle: {lss0_angle_deg}, top boundry: {self.top_lss0}")
    #     self.get_logger().info(f"LSS1, bottom boundary: {self.bottom_lss1}, move angle: {lss1_angle_deg}, top boundry: {self.top_lss1}")
    #     if self.bottom_lss0 >= lss0_angle_deg >= self.top_lss0 and \
    #        self.bottom_lss1 <= lss1_angle_deg <= self.top_lss1:
    #         lss0.move(lss0_position)
    #         lss1.move(lss1_position)
    #         self.get_logger().info(f"Moved LSS0 to {lss0_angle_deg} degrees, LSS1 to {lss1_angle_deg} degrees")
    #     else:
    #         self.get_logger().warn("Received angles are out of bounds")
    #         self.out_of_bounds_publisher.publish(String(data="Angles out of bounds"))
    #     lss0_angle_deg, lss1_angle_deg = angles
    #     target_lss0_position = self.calc_position(lss0_angle_deg)
    #     target_lss1_position = self.calc_position(lss1_angle_deg)
    #     self.get_logger().info(f"Target positions: LSS0 {target_lss0_position}, LSS1 {target_lss1_position}")

    #     # Creating and starting threads for each servo movement:
    #     thread0 = threading.Thread(target=self.move_servo, args=(lss0, target_lss0_position))
    #     thread1 = threading.Thread(target=self.move_servo, args=(lss1, target_lss1_position))

    #     thread0.start()
    #     thread1.start()
        
    #     thread0.join()
    #     thread1.join()

    # def move_servo(self, servo, angle):
    #     if servo == lss0:
    #         lss0.move(angle)
    #         self.get_logger().info(f"Tried moving LSS0 to angle: {angle}")
    #     elif servo == lss1:
    #         lss1.move(angle) # GPT, why wont this work
    #         self.get_logger().info(f"Tried moving LSS1 to angle: {angle}")

    # def move_servos_to_angles(self, angles):
    #     """ Moves the servos to the specified angles received in a pair [lss0_angle, lss1_angle] """
    #     if len(angles) != 2:
    #         self.get_logger().error("Angle pair does not contain exactly two elements.")
    #         return

    #     lss0_angle_deg = angles[0]
    #     lss1_angle_deg = angles[1]

    #     lss0_position = self.calc_position(lss0_angle_deg)
    #     lss1_position = self.calc_position(lss1_angle_deg)

    #     self.get_logger().info(f"LSS0, bottom boundary: {self.bottom_lss0}, move angle: {lss0_angle_deg}, top boundry: {self.top_lss0}")
    #     self.get_logger().info(f"LSS1, bottom boundary: {self.bottom_lss1}, move angle: {lss1_angle_deg}, top boundry: {self.top_lss1}")
    #     if self.bottom_lss0 >= lss0_angle_deg >= self.top_lss0 and \
    #        self.bottom_lss1 <= lss1_angle_deg <= self.top_lss1:
    #         lss0.move(lss0_position)
    #         lss1.move(lss1_position)
    #         self.get_logger().info(f"Moved LSS0 to {lss0_angle_deg} degrees, LSS1 to {lss1_angle_deg} degrees")
    #     else:
    #         self.get_logger().warn("Received angles are out of bounds")
    #         self.out_of_bounds_publisher.publish(String(data="Angles out of bounds"))


    def start_test_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting test of motor movement')
            self.test_motors()

    def manualy_callback(self):
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
            self.manualy_callback()    

    def read_ref_point_callback(self, msg):
        if msg.data == "start":
            lss0.limp()
            lss1.limp()
            self.ref_point_read_and_pub_servo_angles()

    def state_callback(self, msg):
        self.state = msg.data

    def check_state_and_stop(self):
        """Check the current state and stop movement if it's changed to standby."""
        if self.state == 'standby':
            # Stop the movement of servos
            for servo_key in self.servos:
                self.angles_queues[servo_key].queue.clear()  # Clear the angles queue
                self.servos[servo_key].wheelRPM(0)  # Stop the servo
            self.completed_movements = 0
            self.total_movements = 0


def main(args=None):
   rclpy.init(args=args)
   motor_controller = motorControl()
   rclpy.spin(motor_controller)
   rclpy.shutdown()


if __name__ == '__main__':
    main()