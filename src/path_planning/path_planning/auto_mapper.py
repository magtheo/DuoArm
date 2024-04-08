#!/usr/bin/env python3
# auto_mapper.py

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.optimize import fsolve
import json
import time
from .equation import equation, D, initial_guesses, grid_size
from path_planning import StartMapping

# Define the lengths of the robot arm segments
LL1, LL2 = 20, 30  # Left arm segment lengths in cm
LR1, LR2 = 30, 30  # Right arm segment lengths in cm
W = 20             # Distance between the base joints in cm



class AutoMapper(Node):

    # Define angle limits
    MIN_THETA1_LEFT = np.radians(-13)
    MAX_THETA1_LEFT = np.radians(90)
    MIN_THETA1_RIGHT = np.radians(90)
    MAX_THETA1_RIGHT = np.radians(13)

    def __init__(self):
        super().__init__('auto_mapper')
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'actual_joint_angles',
            self.joint_angles_callback,
            10)
        self.joint_state_msg = None
        self.mapping = {}  # To store the mapped coordinates with joint angles
        
        # Initialize the publisher for sending joint angles
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            'calculated_joint_angles',
            10
        )

        # Subscriber for receiving start mapping command
        self.start_mapping_sub = self.create_subscription(
            String,
            'start_mapping',
            self.start_mapping_callback,
            10)
        
        # Publishing mapping completion notification
        self.mapping_done_pub = self.create_publisher(String, 'mapping_done', 10)

    def map_workspace(self, initial_guesses):
        for x in range(0, grid_size, 1): 
            for z in range(0, grid_size, 1):

                print(f'x{x}, z{z}')

                # calculate joint angles with position (x, z)
                theta1_left, theta1_right = self.solve_IK(x, z, initial_guesses)
                print(f'tehtaleft: {theta1_left}, theta_right: {theta1_right}')
                
                self.send_calculated_joint_angles(theta1_left, theta1_right)

                # Wait for robot to reach the position and stabilize
                self.wait_until_stable(theta1_left, theta1_right)

                if self.check_angles_within_limits(theta1_left, theta1_right):
                    print(f"Coordinate ({x}, {z}) is inside the work area.")
                    self.mapping[f"{x},{z}"] = (theta1_left, theta1_right, 'inside')
                else:
                    print(f"Coordinate ({x}, {z}) is outside the work area.")
                    self.mapping[f"{x},{z}"] = (theta1_left, theta1_right, 'outside')

                # redundant
                # if self.joint_state_msg:
                #     theta_left, theta_right = self.read_joint_angles(self.joint_state_msg)
                #     self.robot_arm.add_mapping(x, z, theta_left, theta_right)

    def check_angles_within_limits(self, theta1_left, theta1_right):
        # Check if the angles are within the specified limits
        return (self.MIN_THETA1_LEFT <= theta1_left <= self.MAX_THETA1_LEFT and
                self.MIN_THETA1_RIGHT <= theta1_right <= self.MAX_THETA1_RIGHT)

    def joint_angles_callback(self, msg):
        actual_joint_angles = msg
        self.get_logger().info(f'Received a joint angles {actual_joint_angles}')

    def wait_until_stable(self, theta_left, theta_right):
        flag = False
        while flag == False:
            """Waits a given period for the robot to reach the position and stabilize."""
            self.get_logger().info('Waiting for the robot to stabilize...')
            time.sleep(0.5)  # Waits for 0.5 seconds
            
            # commanded_angles is a list with the calculated angles, 
            # like [theta1_left, theta1_right], in radians.
            commanded_angles = [theta_left, theta_right]
            print(commanded_angles)

            # Ensure that joint_state_msg is not None and has enough positions
            if self.joint_state_msg is None or len(self.joint_state_msg.position) < 2:
                self.get_logger().error('Insufficient joint state data.')
                return False
            
            # actual_angles will be read from the robot's joint_state message.
            # Assuming that positions 0 and 1 correspond to theta1_left and theta1_right.
            actual_angles = [self.joint_state_msg.position[0], self.joint_state_msg.position[2]]
            print(actual_angles)

            # TESTING
            # actual_angles = commanded_angles # Testing
            # Testing

            # Check if the actual angles are close enough to the commanded angles
            tolerance = np.radians(5)  # 5 degree tolerance in radians. TODO play with this number
            if all(np.isclose(commanded_angles, actual_angles, atol=tolerance)):
                self.get_logger().info('Robot has stabilized at the target position.')
                flag = True
            else:
                self.get_logger().warn('Robot has not stabilized at the target position.')
                return False
            

    def solve_IK(self, x, z, initial_guesses):
        # Convert the (x, z) position to joint angles using IK

        # Solve the equations using fsolve
        solution = fsolve(equation, initial_guesses, args=(x, z, D), full_output=True)
     
        # fsolve returns a tuple where the first element is the solution
        # and the fourth element is an integer flag indicating if a solution was found
        solution_values, infodict, ier, mesg = solution

        # Check the solution
        if not ier:
            self.get_logger().error(f'IK solution not found: {mesg}')
            return

        # Extract the joint angles in radians from the solution
        theta1_left, theta2_left, theta1_right, theta2_right = solution_values

        """
        theta1_left: This is the angle of the joint connecting the left arm to the horizontal base. measured counterclockwise.

        theta2_left: This is the angle of the second joint of the left arm. It's the angle between the first arm segment (LL1) and the second segment (LL2), again measured counterclockwise. This angle describes the "elbow" bend of the left arm.

        theta1_right: This is the angle of the first joint of the right arm. Like theta1_left, it measures the orientation of the first segment (LR1) of the right arm relative to the horizontal base or reference line.

        theta2_right: This is the angle of the second joint of the right arm, the "elbow" angle. It measures how the second segment (LR2) of the right arm bends relative to the first segment.
        """
        
        return theta1_left, theta1_right
    
    def send_calculated_joint_angles(self, theta1_left, theta1_right):
        
        # Create a message with the desired joint angles
        msg = Float64MultiArray()
        msg.data = [theta1_left, theta1_right]        
        
        # Publish the message to the controller topic
        # The topic and message type might be different for your setup
        self.joint_command_publisher.publish(msg)
        self.get_logger().info('Published joint angles to move robot to position')

        return msg.data

    def read_joint_angles(self, joint_state_msg):
        # Extract joint angles from the joint_state_msg
        # This is simplified; you'd extract the specific joint angles you need
        theta_left = joint_state_msg.position[0]
        theta_right = joint_state_msg.position[1]
        return theta_left, theta_right
    
    def save_mappings_to_file(self, filename):
        """Saves the mapping to a JSON file."""
        with open(filename, 'w') as file:
            json.dump(self.mapping, file, indent=4)
        self.get_logger().info(f'Saved mappings to {filename}')
        print( "mapping saved to: "+ filename)

    def map_workspace_and_save(self, filename):
        """Maps the workspace and saves the mappings to a file."""
        self.map_workspace(initial_guesses)
        self.save_mappings_to_file(filename)
        self.mapping_done_pub.publish(String(data="done")) # Notify that mapping is done

    def start_mapping_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info('Starting workspace mapping')
            self.map_workspace_and_save('robot_arm_mappings.json')

        
    
def main(args=None):
    rclpy.init(args=args)
    auto_mapper = AutoMapper()
    
    filename = 'robot_arm_mappings.json'  # Define your filename here
    
    # Initiate mapping
    #auto_mapper.map_workspace_and_save(filename)
    
    rclpy.spin(auto_mapper)
    rclpy.shutdown()


# Ensure the main function is called when the script is executed
if __name__ == '__main__':
    main()






