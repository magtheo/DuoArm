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
    """
    @class motorControl
    @brief Used to controll and read values form LSS servo motor during mapping and path execution

    The motorControl node is designed to interface with LSS servos and manage their operations
    through various topics. This node handles real-time control, state management,
    and boundary safety for motor operations within this ROS system.
    """
    def __init__(self):
        """Initialize the motor control node, setting up publishers, subscribers, and servo control threads."""
        super().__init__("motor_control")

        # Define a QoS profile for real-time updates
        self.real_time_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        
        # Initialize other attributes and subscriptions...
        self.boundaries = {}
        self.load_and_set_boundaries()

        self.setup_servos_and_queues()

        self.threads = []
        self.init_servo_threads()

        self.init_communication()

        self.lock = threading.Lock()
        self.current_target = None


        self.state = 'standby'

        self.total_movements = 0
        self.completed_movements = 0
        self.current_movement_nr = 0

    def init_communication(self):
        """
        Initialize communication for the node.
        Setup all necessary publishers and subscribers for sending and receiving commands
        and data to and from other components of ROS system.
        """
        self.actual_joint_angles_publisher = self.create_publisher(
            Float64MultiArray, 
            'actual_joint_angles', # publish the read angels, used during mapping and motor control
            10
        )

        self.joint_angles_publisher = self.create_publisher(
            String,
            'joint_angles',  # Topic to publish the joint angles to display node, not used as we didnt get the display node to work properly
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

        self.limp_and_reset_origin_sub = self.create_subscription(
            String,
            'limp_and_reset_origin',
            self.limp_and_set_origin,
            10)
        
        self.read_angels_sub = self.create_subscription(
            String, 'read_angles', self.read_angles_callback, 10)
        
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray, 'joint_angles_array', self.follow_path_callback, 10)

        self.path_done_pub = self.create_publisher(
            String, 'system_state_request', 10)
        
        self.state_subscription = self.create_subscription(
            String, 'system_state', self.state_callback, 10)
  
    def load_and_set_boundaries(self):
        """
        Load and set boundaries for servo movements from a JSON configuration file.
        Ensures that servos operate within safe operational limits to avoid mechanical damage.
        """
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
        """
        Initializes servo objects and angles queues for asynchronous operation.
        """
        self.servos = {'lss0': LSS(0), 'lss1': LSS(1)}
        self.angles_queues = {key: queue.Queue() for key in self.servos}

    def init_servo_threads(self):
        """Initialize threads for controlling servos independently."""
        for key in self.servos:
            thread = threading.Thread(target=self.servo_control_loop, args=(key,))
            thread.start()

    def servo_control_loop(self, servo_key):
        """
        Control loop for servos.
        Handles target angles for a single servo from its angles queue.
        """
        while True:
            target_angle = self.angles_queues[servo_key].get()
            with self.lock:
                self.completed_movements += 1
            self.control_servo_speed(servo_key, target_angle)


    def control_servo_speed(self, servo_key, target_angle):
        """
        Control the speed of a servo to reach a specified target angle.
        Implements PID control to reach the target angle by comparing current and target angle.
        """
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
                
                self.publish_joint_angles(servo_key, current_angle)  # Publish the current angles to display
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


    def follow_path_callback(self, msg):
        """
        Callback for moving servos according to a set of predefined angles.
        Handles the reception of an array of angles, setting the path for the servos to follow, and number of times to iterate over the path.
        """
        self.get_logger().info("motor_controller received array of angles")
        angles = np.array(msg.data)
        num_cycles = 3  # Number of times to repeat the path back and forth
        self.follow_path(angles, num_cycles)
        

    def follow_path(self, angles, num_cycles):
        """
        Reads the target angles and puts them in the queue for independent servo threads to read
        Keeps track of movement iteration and correctly ending a path execution

        - explination of reverse for loop - 
        len(angles) - 2: This starts the index from the second-to-last element in the angles list. It adjusts for Python's zero-based indexing, ensuring the arm starts reversing from the correct position.

        -1: This is the end parameter of the range, which is exclusive. In Python's range, when counting backwards, you need to go one past the first index you want to include, hence -1 is used to ensure it includes the index 0.

        -2: This is the step parameter, which determines the increment between each step in the range. Here, -2 means the loop decrements the index by 2 each time, effectively moving backwards through the angles list two elements at a time. This allows the system to address pairs of angles (assuming each servo position in the pair is spaced by one index in the list) in reverse order
        """
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
        self.get_logger().info(f"published done to state_manager")

    
    def limp_and_set_origin(self, msg):
        """
        Makes the servos go limp and sets a new origin after a delay.
        Used at the start of mapping process
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


    def publish_joint_angles(self, servo_key, angle):
        """
        Publish the current joint angles to display node for visualization, sadly didnt get it working.
        """
        data = {'servo': servo_key, 'angle': angle}
        msg = String()
        msg.data = json.dumps(data)
        self.joint_angles_publisher.publish(msg)
        self.get_logger().info(f"Published current angles for {servo_key}")
        
    
    def calc_position(self, angle):
        """
        Calculate the joint position. position is LSS servomotors way of describing angle, position = angle*10 
        this is used when moving the servos
        """
        return angle*10
    
    def calc_angle(self, position):
        """
        Calculate the joint angle based on the servo position reading. position is LSS servomotors way of describing angle, angle = position/10 
        Converts raw position data from the servo to a human-readable angle format.
        """
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


    def read_angles_callback(self):
        """
        Read the servos when during mapping, is called when arm state='map' and map button is pressed
        """
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

    
    def manualy_callback(self):
        """
        Read the servos when during testing and debugging
        """
        self.get_logger().info( 'test ')
        lss0.limp()
        lss1.limp()

        for i in range(2):
            self.get_logger().info(f'current loop: {i}')

            time.sleep(2)

            self.get_logger().info(f'lss0, right: {self.calc_angle(lss0.getPosition())}')
            self.get_logger().info(f'lss1, left: {self.calc_angle(lss1.getPosition())}')

    def start_read_callback(self, msg):
        """
        callback for starting test reading
        """
        if msg.data == "start":
            self.get_logger().info('Starting to read motor angles')
            self.manualy_callback()    



    def check_state_and_stop(self):       
        """
        Check the system's state and stop all servo movements if necessary.
        sadly we couldnt get the node to recive messages whils operating the servos, and therefor this funciton is useless atm.
        """     
        self.get_logger().info('enterd check state and stop predefined path')
        if self.state == 'standby':
            # Stop the movement of servos
            for servo_key in self.servos:
                self.get_logger().info('stop path, for loop iteration')
                self.angles_queues[servo_key].queue.clear()  # Clear the angles queue
                self.servos[servo_key].wheelRPM(0)  # Stop the servo
            self.completed_movements = 0
            self.total_movements = 0

    def state_callback(self, msg):
        """
        Callback for updating the system's operational state.
        Receives state updates from the action controller.
        """
        self.state = msg.data
        self.get_logger().info(f'motor controller state updated: {self.state}')



def main(args=None):
    """
    Main function to initialize and run the motor control node.
    """
    rclpy.init(args=args)
    motor_controller = motorControl()
    rclpy.spin(motor_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()