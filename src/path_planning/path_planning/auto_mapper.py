#!/usr/bin/env python3
# auto_mapper.py

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from scipy.optimize import fsolve
import json
import time
from .equation import equation, D, grid_size_x, grid_size_z


# Define the lengths of the robot arm segments
LL1, LL2 = 20, 30  # Left arm segment lengths in cm
LR1, LR2 = 20, 30  # Right arm segment lengths in cm
W = 20             # Distance between the base joints in cm


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')

        self.actual_joint_angles = None
        self.actual_joint_angle_flag = False

        self.current_point_index = 0  # Tracks the index of the current grid point being processed
        self.is_point_processed = True  # Flag to indicate when the robot has finished processing a point

        self.mapping = {}  # To store the mapped coordinates with joint angles
        self.calculated_angles = []
        self.actual_joint_angles = []

       
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'actual_joint_angles',
            self.joint_angles_callback,
            10)
        
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
            10
            )
        
        # Publishing mapping completion notification
        self.mapping_done_pub = self.create_publisher(
            String,
            'mapping_done',
            10
            )

        # Define angle limits
        self.MIN_THETA1_LEFT = np.radians(-13)
        self.MAX_THETA1_LEFT = np.radians(90)
        self.MIN_THETA1_RIGHT = np.radians(-90)
        self.MAX_THETA1_RIGHT = np.radians(13)

        # Manually determined reference angles for the top and bottom center points
        self.ref_angles_top = [self.MIN_THETA1_LEFT, np.radians(45), self.MAX_THETA1_RIGHT, np.radians(-45)] 
        self.ref_angles_bottom = [self.MAX_THETA1_LEFT, np.radians(175), self.MIN_THETA1_RIGHT, np.radians(-175)]
        self.ref_angles_left = []

        # Define new reference angles for max/min X at center Z
        self.ref_angles_max_x = [theta1_left_max_x, theta2_left_max_x, theta1_right_max_x, theta2_right_max_x]  # Replace with your actual angles
        self.ref_angles_min_x = [theta1_left_min_x, theta2_left_min_x, theta1_right_min_x, theta2_right_min_x]  # Replace with your actual angles


    def prepare_grid_points(self):
        # Initialize an empty list to hold the grid points and initial guesses
        self.grid_points = []

        # Assume grid origin (0,0) is at the bottom left
        ref_x_center = grid_size_x / 2
        ref_z_center = grid_size_z / 2  # Center of the grid in Z
        ref_z_top = grid_size_z  # Top of the grid in Z

        # Iterate over the grid in x and z dimensions
        for x in range(grid_size_x): 
            for z in range(grid_size_z):
                if x == ref_x_center and z == ref_z_top:
                    current_guesses = self.ref_angles_top.copy()
                elif x == ref_x_center and z == 0:
                    current_guesses = self.ref_angles_bottom.copy()
                elif x == 0 and z == ref_z_center:
                    current_guesses = self.ref_angles_min_x.copy()
                elif x == grid_size_x - 1 and z == ref_z_center:
                    current_guesses = self.ref_angles_max_x.copy()
                else:
                    # As a default, perhaps take an average of the reference angles
                    current_guesses = np.mean([self.ref_angles_top, self.ref_angles_bottom, 
                                            self.ref_angles_min_x, self.ref_angles_max_x], axis=0).tolist()

                # Append the tuple (x, z, current_guesses) to the list
                self.grid_points.append((x, z, current_guesses))

        self.get_logger().info(f'Prepared {len(self.grid_points)} grid points for mapping.')

    def process_grid_point(self):
        if self.current_point_index >= len(self.grid_points):
            self.save_mappings_to_file()
            self.get_logger().info('All points have been processed.')
            self.mapping_done_pub.publish(String(data="done"))  # Notify that mapping is done
            return

        # Get the current grid point
        self.x, self.z, _ = self.grid_points[self.current_point_index]

        # Dynamically determine initial guesses based on the target point's coordinates
        initial_guesses = self.dynamic_initial_guesses(self.x, self.z)

        # Solve the IK for this grid point using the dynamically determined initial guesses
        theta1_left, theta2_left, theta1_right, theta2_right = self.solve_IK(self.x, self.z, initial_guesses)

        # Sending calculated angels to motorController, this initates the motors to move to these angles
        self.send_calculated_joint_angles(theta1_left, theta1_right)



    def dynamic_initial_guesses(self, x, z):
        # Calculate distances to each reference point
        distance_to_top = abs(z - grid_size_z)  # Distance to the top reference point
        distance_to_bottom = abs(z)  # Distance to the bottom reference point
        distance_to_max_x = abs(x - grid_size_x)  # Distance to the max X reference point
        distance_to_min_x = abs(x)  # Distance to the min X reference point
        
        # Determine the closest reference point based on the minimum distance
        min_distance = min(distance_to_top, distance_to_bottom, distance_to_max_x, distance_to_min_x)
        
        # Choose the initial guesses based on the closest reference point
        if min_distance == distance_to_top:
            return self.ref_angles_top.copy()
        elif min_distance == distance_to_bottom:
            return self.ref_angles_bottom.copy()
        elif min_distance == distance_to_max_x:
            return self.ref_angles_max_x.copy()
        elif min_distance == distance_to_min_x:
            return self.ref_angles_min_x.copy()
        else:
            # As a default, use a weighted average of the reference angles
            # You could also use more sophisticated methods like a weighted sum based on distances to each reference point
            return np.mean([self.ref_angles_top, self.ref_angles_bottom, 
                            self.ref_angles_min_x, self.ref_angles_max_x], axis=0).tolist()



    def map_point(self, actual_joint_angles):
        actual_theta_left, actual_theta_right = actual_joint_angles
        # Store the mapping
        if self.check_angles_within_limits(actual_theta_left, actual_theta_right):
            self.mapping[f"{self.x},{self.z}"] = (actual_theta_left, actual_theta_right, 'inside')
        else:
            self.mapping[f"{self.x},{self.z}"] = (actual_theta_left, actual_theta_right, 'outside')

        self.current_point_index += 1  # Prepare for the next point

        self.process_grid_point()


    def map_workspace(self, initial_guesses):   
        # Assume grid origin (0,0) is at the bottom left
        ref_x = grid_size_x / 2
        ref_y = grid_size_z  # Top of the grid

        # Set initial guesses to the reference angles for the bottom center
        current_guesses = self.ref_angles_bottom.copy()

        for x in range(0, grid_size_x + 1): 
            for z in range(0, grid_size_z + 1):

                print(f'x{x}, z{z}')

                if z == ref_y and x == ref_x:
                    current_guesses = self.ref_angles_top.copy()

                # calculate joint angles with position (x, z)
                theta1_left, theta2_left, theta1_right, theta2_right = self.solve_IK(x, z, current_guesses)
                print(f'tehtaleft: {theta1_left}, theta_right: {theta1_right}')
                
                # Store the current solution to use as initial guesses for the next point
                current_guesses = [theta1_left, theta2_left, theta1_right, theta2_right]

                self.send_calculated_joint_angles(theta1_left, theta1_right)

                # Reset the flag before waiting for new data
                self.actual_joint_angle_flag = False

                # Wait for robot to reach the position and stabilize
                while not self.actual_joint_angle_flag:
                    time.sleep(0.1)
                
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
        self.actual_joint_angles = msg.data
        self.get_logger().info(f'Received actual joint angles: {self.actual_joint_angles}')
        self.map_point(self.actual_joint_angles)



    def wait_until_stable(self, theta_left, theta_right):
        flag = False
        while flag == False:
            """Waits a given period for the robot to reach the position and stabilize."""
            self.get_logger().info('Waiting for the robot to stabilize...')
            time.sleep(1)  # Waits for 1 seconds
            
            # calculated_angles is a list with the calculated angles, 
            # like [theta1_left, theta1_right], in radians.
            calculated_angles = [theta_left, theta_right]
            self.get_logger().info(f'calculated angles. DATA: {calculated_angles}')

            # Ensure that joint_state_msg is not None and has enough positions
            if self.actual_joint_angles is None:
                self.get_logger().error(f'Insufficient joint state data. DATA: {self.actual_joint_angles}')
            
            # actual_angles will be read from the robot's joint_state message.
            # Assuming that positions 0 and 1 correspond to theta1_left and theta1_right.
            actual_angles = [self.actual_joint_angles.position[0], self.actual_joint_angles.position[2]]
            
            # TESTING
            # actual_angles = calculated_angles # Testing
            # Testing

            # Check if the actual angles are close enough to the commanded angles
            tolerance = np.radians(5)  # 5 degree tolerance in radians. TODO play with this number
            if all(np.isclose(calculated_angles, actual_angles, atol=tolerance)):
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
        
        return theta1_left, theta2_left, theta1_right, theta2_left
    
    def send_calculated_joint_angles(self, theta1_left, theta1_right):
        
        # Create a message with the desired joint angles
        msg = Float64MultiArray()
        msg.data = [theta1_left, theta1_right]        
        
        # Publish the message to the controller topic
        # The topic and message type might be different for your setup
        self.joint_command_publisher.publish(msg)
        self.get_logger().info('Published calculated joint angles during mapping')

        return msg.data

    def read_joint_angles(self, joint_state_msg):
        # Extract joint angles from the joint_state_msg
        # This is simplified; you'd extract the specific joint angles you need
        theta_left = joint_state_msg.position[0]
        theta_right = joint_state_msg.position[1]
        return theta_left, theta_right
    
    def save_mappings_to_file(self):
        filename = 'robot_arm_mappings.json'
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
            self.prepare_grid_points()  # This pre-computes the grid points and initial guesses
            self.process_grid_point()

        
    
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






