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
from .equation import equation, D

# Define the lengths of the robot arm segments
LL1, LL2 = 20, 30  # Left arm segment lengths in cm
LR1, LR2 = 30, 30  # Right arm segment lengths in cm
W = 20             # Distance between the base joints in cm
D = 10             # Distance between the tool hub joints in cm

grid_size = 100 # bruk 100

class AutoMapper(Node):
    
    def __init__(self):
        super().__init__('auto_mapper')
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'actual_joint_states',
            self.joint_state_callback,
            10)
        self.joint_state_msg = None
        self.mapping = {}  # To store the mapped coordinates with joint angles
        
        # Initialize the publisher for sending joint angles
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            'calculated_joint_angles',
            10
        )

    def map_workspace(self):
        for x in range(0, grid_size, 1):  # Grid size
            for z in range(0, grid_size, 1):

                print(f'x{x}, z{z}')

                # Command robot to move to the position (x, z)
                theta1_left, theta1_right = self.solve_IK(x, z)
                print(f'tehtaleft: {theta1_left}, theta_right: {theta1_right}')
                
                self.send_calculated_joint_angles(theta1_left, theta1_right)

                # Wait for robot to reach the position and stabilize
                self.wait_until_stable(theta1_left, theta1_right)

                # Record the joint states
                if theta1_left is not None and theta1_right is not None:
                # Update the mapping with the new data
                    #self.mapping[(x, z)] = (theta_left, theta_right)
                    self.mapping[f"{x},{z}"] = (theta1_left, theta1_right)

                # if self.joint_state_msg:
                #     theta_left, theta_right = self.read_joint_angles(self.joint_state_msg)
                #     self.robot_arm.add_mapping(x, z, theta_left, theta_right)


    def joint_state_callback(self, msg):
        self.joint_state_msg = msg
        self.get_logger().info('Received a joint state message')

    def wait_until_stable(self, theta_left, theta_right):
        flag = False
        while flag == False:
            """Waits a given period for the robot to reach the position and stabilize."""
            self.get_logger().info('Waiting for the robot to stabilize...')
            #time.sleep(0.1)  # Waits for 0.5 seconds
            
            # commanded_angles is a list with the calculated angles, 
            # like [theta1_left, theta1_right], in radians.
            commanded_angles = [theta_left, theta_right]
            print(commanded_angles)

            # Ensure that joint_state_msg is not None and has enough positions
            # if self.joint_state_msg is None or len(self.joint_state_msg.position) < 2:
            #     self.get_logger().error('Insufficient joint state data.')
            #     return False
            
            # actual_angles will be read from the robot's joint_state message.
            # Assuming that positions 0 and 1 correspond to theta1_left and theta1_right.
            # actual_angles = [self.joint_state_msg.position[0], self.joint_state_msg.position[2]]
            # print(actual_angles)

            # TESTING
            actual_angles = commanded_angles
            # Testing

            # Check if the actual angles are close enough to the commanded angles
            tolerance = np.radians(5)  # 5 degree tolerance in radians. TODO play with this number
            if all(np.isclose(commanded_angles, actual_angles, atol=tolerance)):
                self.get_logger().info('Robot has stabilized at the target position.')
                flag = True
            else:
                self.get_logger().warn('Robot has not stabilized at the target position.')
                return False
            

    def solve_IK(self, x, z):
        # Convert the (x, z) position to joint angles using IK

        # Initial guesses for theta1 and theta2 for both arms, in degrees
        initial_guesses_degrees = (30, 90, -30, 90) # TODO test whether these numbers are accurate
        # theta1_left, theta2_left, theta1_right, theta2_right

        # Convert initial guesses from degrees to radians
        initial_guesses_radians = np.radians(initial_guesses_degrees)
        
        # Solve the equations using fsolve
        solution = fsolve(equation, initial_guesses_radians, args=(x, z, D), full_output=True)
     
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
        self.map_workspace()
        self.save_mappings_to_file(filename)
    
    
def main(args=None):
    rclpy.init(args=args)
    auto_mapper = AutoMapper()
    
    filename = 'robot_arm_mappings.json'  # Define your filename here
    
    # Initiate mapping
    auto_mapper.map_workspace_and_save(filename)
    
    rclpy.spin(auto_mapper)
    rclpy.shutdown()


# Ensure the main function is called when the script is executed
if __name__ == '__main__':
    main()






