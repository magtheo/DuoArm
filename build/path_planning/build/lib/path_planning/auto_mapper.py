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
from .equation import equation_offset, D, W, grid_size_x, grid_size_z
import threading
import tkinter as tk
from queue import Queue

# from controll_center.srv import MoveServos


# Define the lengths of the robot arm segments
LL1, LL2 = 12, 27  # Left arm segment lengths in cm
LR1, LR2 = 12, 27  # Right arm segment lengths in cm


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

        self.mapping_done = False

        self.ref_points = ['top', 'bottom', 'max_x', 'min_x']
        self.current_ref_point_index = 0  # Start with the first reference point

        # Initialize the communication queue
        self.gui_queue = Queue()

        # GUI related initialization method
        self.init_gui()

       
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'actual_joint_angles',
            self.joint_angles_callback,
            10
            )
        
        # Initialize the publisher for sending joint angles
        self.joint_command_publisher = self.create_publisher(
            Float64MultiArray,
            'calculated_joint_angles',
            10
            )
        
        # Topic for reading arm state from action controller
        self.state_subscription = self.create_subscription(
            String,
            'action_controller_state',
            self.state_callback,
            10)
        
        self.arm_state = 'standby'  # Initialize with the default state

        
        # Publishing mapping completion notification
        self.mapping_done_pub = self.create_publisher(
            String,
            'mapping_done',
            10
        )
        
        self.start_read_publisher = self.create_publisher(
            String,
            'start_ref_read',
            10
        )
        self.manual_readings_sub = self.create_subscription(
            Float64MultiArray,
            'manual_angle_readings',
            self.manual_readings_callback,
            10
        )
        
        self.arm_position_pub = self.create_publisher(
            String,
            'arm_position',
            10
        )

        self.grid_data_publisher = self.create_publisher(
            String,
            'grid_data',
            10)
        
        # self.servo_client = self.create_client(MoveServos, 'move_servos')
        # while not self.servo_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service move_servos not available, waiting again...')

        # Define angle limits
        self.MIN_THETA1_LEFT = np.radians(160)
        self.MAX_THETA1_LEFT = np.radians(270)
        self.MIN_THETA1_RIGHT = np.radians(-90)
        self.MAX_THETA1_RIGHT = np.radians(40)

        # Manually determined reference angles for the top and bottom center points
        self.ref_angles_top = [self.MIN_THETA1_LEFT, np.radians(45), self.MAX_THETA1_RIGHT, np.radians(-45)] 
        self.ref_angles_bottom = [self.MAX_THETA1_LEFT, np.radians(175), self.MIN_THETA1_RIGHT, np.radians(-175)]

        # angels for ref point in max x
        self.theta1_left_max_x = np.radians(-185)
        self.theta2_left_max_x = np.radians(110)
        self.theta1_right_max_x = np.radians(-50)
        self.theta2_right_max_x = np.radians(90)
        
        # angels for ref point in min x
        self.theta1_left_min_x = np.radians(19)
        self.theta2_left_min_x = np.radians(90)
        self.theta1_right_min_x = np.radians(170)
        self.theta2_right_min_x = np.radians(120)

        # Define new reference angles for max/min X at center Z
        self.ref_angles_max_x = [self.theta1_left_max_x, self.theta2_left_max_x, self.theta1_right_max_x, self.theta2_right_max_x]  # Replace with your actual angles
        self.ref_angles_min_x = [self.theta1_left_min_x, self.theta2_left_min_x, self.theta1_right_min_x, self.theta2_right_min_x]  # Replace with your actual angles


        #new ref point method
        self.ref_point_top = (grid_size_x // 2, grid_size_z - 1)  # Top center
        self.ref_point_bottom = (grid_size_x // 2, 0)  # Bottom center
        self.ref_point_max_x = (grid_size_x - 1, grid_size_z // 2)  # Max X, center Z
        self.ref_point_min_x = (0, grid_size_z // 2)  # Min X, center Z

        self.ref_points_coordinates = {
            'top': self.ref_point_top,
            'bottom': self.ref_point_bottom,
            'max_x': self.ref_point_max_x,
            'min_x': self.ref_point_min_x
        }

        self.ref_points_angles = {}

    # def request_target_movement(self, target_angles):
    #     req = MoveServos.Request()
    #     req.target_angles = target_angles
    #     future = self.cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     if future.result() is not None:
    #         actual_angles = future.result().actual_angles
    #         success = future.result().success
    #         message = future.result().message
    #         # Process the response
    #         self.map_point(actual_angles)
    #         return actual_angles, success, message
    #     else:
    #         self.get_logger().error('Service call failed')
    #         return None, False, 'Failed'

    def prepare_grid_points(self):
        self.grid_points = []
        for x in range(grid_size_x):
            for z in range(grid_size_z):
                point_type = 'normal'
                if (x, z) == self.ref_point_top:
                    point_type = 'ref_top'
                elif (x, z) == self.ref_point_bottom:
                    point_type = 'ref_bottom'
                elif (x, z) == self.ref_point_max_x:
                    point_type = 'ref_max_x'
                elif (x, z) == self.ref_point_min_x:
                    point_type = 'ref_min_x'
                
                self.grid_points.append({"coords": (x, z), "type": point_type, "initial_guesses": []})

        self.get_logger().info(f'Prepared {len(self.grid_points)} grid points for mapping.')

        # Publish generated grid to display node
        grid_data = [{'x': point['coords'][0], 'z': point['coords'][1], 'type': point['type']} for point in self.grid_points]
        self.grid_data_publisher.publish(String(data=json.dumps(grid_data)))
        self.get_logger().info('Published grid data.')

        # Print ref point to map
        self.get_logger().info(f'first ref point to map: {list(self.ref_points_coordinates.keys())[0]}')


    def process_grid_point_OLD(self):
        
        # Is the mapping done?
        if self.current_point_index >= len(self.grid_points):
            self.save_mappings_to_file()
            self.get_logger().info('All points have been processed.')
            self.mapping_done_pub.publish(String(data="done"))  # Notify that mapping is done
            self.mapping_done = True
            return
        
        self.get_logger().info(f'-----------------------------------')
        self.get_logger().info(f'Processing point: {self.current_point_index}')

        # Get the current grid point
        self.x, self.z, _ = self.grid_points[self.current_point_index]

        # Check if the point is within the workspace
        if not self.is_point_within_workspace(self.x, self.z):
            # Mark the point as outside and move to the next point
            self.mapping[f"{self.x},{self.z}"] = "outside"
            self.get_logger().info(f'mapped point ({self.x}, {self.z}) as outside, {self.current_point_index}/{len(self.grid_points)}')

            self.current_point_index += 1
            self.process_grid_point()  # Proceed to the next point
        else:
            
            # Dynamically determine initial guesses based on the target point's coordinates
            initial_guesses = self.dynamic_initial_guesses(self.x, self.z)

            # Solve the IK for this grid point using the dynamically determined initial guesses
            theta1_left, theta2_left, theta1_right, theta2_right = self.solve_IK(self.x, self.z, initial_guesses)
            self.get_logger().info(f'calculated angles: {np.rad2deg(theta1_left)}  { np.rad2deg(theta1_right)}')


            if not self.is_within_max_min_angles(theta1_left, theta1_right):
                self.mapping[f"{self.x},{self.z}"] = "outside"
                self.get_logger().info(f'Marked point ({self.x}, {self.z}) as outside. OUTOF BOUNDS Skipping to next.')
                self.current_point_index += 1  # Move to the next point
                self.process_grid_point()  # Continue processing
            else:
                # Sending calculated angels to motorController, this initates the motors to move to these angles
                self.send_calculated_joint_angles(theta1_left, theta1_right)

    def process_grid_point(self):
        # Is the mapping done?
        if self.current_point_index >= len(self.grid_points):
            self.save_mappings_to_file()
            self.get_logger().info('All points have been processed.')
            self.mapping_done_pub.publish(String(data="done"))  # Notify that mapping is done
            self.mapping_done = True
            return
        
        self.get_logger().info(f'-----------------------------------')
        self.get_logger().info(f'Processing point: {self.current_point_index}')

        grid_point = self.grid_points[self.current_point_index]
        self.x = grid_point['coords'][0]
        self.z = grid_point['coords'][1]
  
        # Dynamically determine initial guesses based on the target point's coordinates
        initial_guesses = self.dynamic_initial_guesses(self.x, self.z)

        # Solve the IK for this grid point using the dynamically determined initial guesses
        theta1_left, theta2_left, theta1_right, theta2_right = self.solve_IK(self.x, self.z, initial_guesses, D, W, grid_size_x, grid_size_z)



        if not None in [theta1_left, theta2_left, theta1_right, theta2_right]:
            self.get_logger().info(f'calculated angles: {np.rad2deg(theta1_left)}  { np.rad2deg(theta1_right)}')
            # Calculate the base positions
            x_center = grid_size_x / 2
            z_base = grid_size_z
            x_base_left = x_center - W / 2
            x_base_right = x_center + W / 2

            # Calculate the end effector positions
            x_left = x_base_left + LL1 * np.cos(theta1_left) + LL2 * np.cos(theta1_left + theta2_left)
            z_left = z_base + LL1 * np.sin(theta1_left) + LL2 * np.sin(theta1_left + theta2_left)

            x_right = x_base_right + LR1 * np.cos(theta1_right) + LR2 * np.cos(theta1_right + theta2_right)
            z_right = z_base + LR1 * np.sin(theta1_right) + LR2 * np.sin(theta1_right + theta2_right)

            # Package the arm positions into a dictionary
            arm_position_data = {
                "x_base_left": x_base_left,
                "x_base_right": x_base_right,
                "z_base": z_base,
                "x_left_end": x_left,
                "z_left_end": z_left,
                "x_right_end": x_right,
                "z_right_end": z_right
            }

            # Publish the arm positions
            self.arm_position_pub.publish(String(data=json.dumps(arm_position_data)))

        # Check if a valid solution was returned before proceeding
        if None in [theta1_left, theta2_left, theta1_right, theta2_right]:
            self.get_logger().info('No valid IK solution, skipping point.')
            self.current_point_index += 1  # Skip this point
            self.process_grid_point()
            return

        if not self.is_within_max_min_angles(theta1_left, theta1_right):
            self.mapping[f"{self.x},{self.z}"] = "outside"
            self.get_logger().info(f'Marked point ({self.x}, {self.z}) as outside. OUTOF BOUNDS Skipping to next.')
            self.current_point_index += 1  # Move to the next point
            self.process_grid_point()  # Continue processing
        else:
            # SERVICE METHOD
            # self.request_target_movement((theta1_left, theta1_right))
            
            # TOPIC METHOD Sending calculated angels to motorController, this initates the motors to move to these angles
            self.send_calculated_joint_angles(theta1_left, theta1_right)

            arm_position = {"x": self.x, "z": self.z, "theta1_left": theta1_left, "theta1_right": theta1_right}  # sends arm position to display
            self.arm_position_pub.publish(String(data=json.dumps(arm_position)))

    def manual_readings_callback(self, msg):
        manual_angles = msg.data

        # Determine the current reference point being set based on index
        ref_point_name = list(self.ref_points_coordinates.keys())[self.current_ref_point_index]

        # Assign received angles to the current reference point
        self.ref_points_angles[ref_point_name] = manual_angles
        
        self.get_logger().info(f"Reference point {ref_point_name} set with angles: {(manual_angles)}")
        self.current_ref_point_index += 1
        
        if self.current_ref_point_index >= len(self.ref_points):
            self.get_logger().info("All reference points set. Starting automatic mapping...")
            self.process_grid_point()
        else:
            self.get_logger().info(f"Move to the next reference point: {self.ref_points[self.current_ref_point_index]} and press a key.")
    

    def start_read_pub(self):
        """
        Publishes a message to trigger the motor_control node to read and publish current angles.
        """
        msg = String()
        msg.data = "start"
        self.start_read_publisher.publish(msg)
        self.get_logger().info('Published start read message to motor_control.')

    def dynamic_initial_guesses(self, x, z):
        # Define default initial guesses for theta2_left and theta2_right
        default_theta2_L = np.deg2rad(90)
        default_theta2_R = np.deg2rad(270)

        # Check if the current grid point matches any reference point's coordinates
        if any((x, z) == coords for coords in self.ref_points_coordinates.values()):
            # Use the manually set angles for this reference point as initial guesses
            # But ensure theta2_left and theta2_right are set to 45 degrees
            # Assumes self.ref_points_angles[ref_point_name] gives [theta1_left, theta1_right]
            ref_point_name = next(name for name, coords in self.ref_points_coordinates.items() if (x, z) == coords)
            initial_theta1_left, initial_theta1_right = self.ref_points_angles.get(ref_point_name, [0, 0])  # Provide a default value to avoid KeyError
            return [initial_theta1_left, default_theta2_L, initial_theta1_right, default_theta2_R]
        
        # Fallback for non-reference points
        # Calculate averages or use another method for initial_theta1_left and initial_theta1_right
        initial_theta1_left = np.mean([self.MIN_THETA1_LEFT, self.MAX_THETA1_LEFT])
        initial_theta1_right = np.mean([self.MIN_THETA1_RIGHT, self.MAX_THETA1_RIGHT])
        return [initial_theta1_left, default_theta2_L, initial_theta1_right, default_theta2_R]




    def map_point(self, actual_joint_angles):
        actual_theta_left, actual_theta_right = actual_joint_angles
        # Store the mapping

        self.mapping[f"{self.x},{self.z}"] = (actual_theta_left, actual_theta_right, 'inside')       
        self.get_logger().info(f'mapped x:{self.x} z:{self.z} as inside workspace with angles:{np.rad2deg(self.actual_joint_angles)}')


        self.get_logger().info(f'mapped point {self.current_point_index}/{len(self.grid_points)}')

        self.current_point_index += 1  # Prepare for the next point

        self.process_grid_point()


    def is_point_within_workspace(self, x, z):
        # Define the center of the workspace
        center_x, center_z = grid_size_x / 2, grid_size_z / 2
        # Calculate the radius of the workspace circle
        workspace_radius = 15  # cm TODO change radius values
        # Calculate the distance of (x, z) from the center
        distance_squared = (x - center_x)**2 + (z - center_z)**2
        # Check if the point is within the circular workspace
        return distance_squared <= workspace_radius**2
    
    def is_within_max_min_angles(self, theta1_left, theta1_right):
        # Checks if the given angles are within the max and min limits
        self.get_logger().info(f'angels in rad ({self.MIN_THETA1_LEFT, theta1_left, self.MAX_THETA1_LEFT}, {self.MIN_THETA1_RIGHT, theta1_right, self.MAX_THETA1_RIGHT}) .')
        return (self.MIN_THETA1_LEFT <= theta1_left <= self.MAX_THETA1_LEFT) and \
               (self.MIN_THETA1_RIGHT <= theta1_right <= self.MAX_THETA1_RIGHT)


    def joint_angles_callback(self, msg):
        if self.mapping_done == False:
            self.actual_joint_angles = msg.data
            self.get_logger().info(f'Received actual joint angles: {np.rad2deg(self.actual_joint_angles)}')
            self.map_point(self.actual_joint_angles)
        else:
            self.get_logger().info(f'---Mapping done---')

    def ref_button_press(self, msg='init'):            
        self.get_logger().info('button pressed')
        self.start_read_publisher.publish(String(data="start"))

        
        # Check if we have processed all reference points
        if self.current_ref_point_index >= len(self.ref_points):
            self.get_logger().info("All reference points set. Starting automatic mapping...")
            self.process_grid_point()
            return
        
        # Inform the user which reference point to move to next
        ref_point_name = list(self.ref_points_coordinates.keys())[self.current_ref_point_index]
        self.get_logger().info(f"Move to the reference point: {ref_point_name} and press the button after positioning.")

    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f'AutoMapper current state: {self.current_state}')
        if self.current_state == 'map':
            self.get_logger().info('Starting mapping process...')
            self.prepare_grid_points()
            # self.process_grid_point()


    def publish_start_ref_read(self):
        self.start_read_publisher.publish(String(data="start"))
        self.get_logger().info("Published start for reference point reading.")

    def init_gui(self):
        # This method initializes the GUI in a way that doesn't block the ROS node
        self.gui_thread = threading.Thread(target=self.run_gui, daemon=True)
        self.gui_thread.start()

    def run_gui(self):
        # Create the main window
        self.root = tk.Tk()
        self.root.title("AutoMapper Control")

        # Create a button for starting manual reference points setting
        tk.Button(self.root, text="Start Manual Ref Points Setting", command=lambda: self.gui_queue.put('start_ref')).pack(pady=20)

        # Start the periodic check of the queue
        self.check_queue()

        # Start the Tkinter event loop
        self.root.mainloop()

    def check_queue(self):
        try:
            while not self.gui_queue.empty():
                message = self.gui_queue.get_nowait()
                # Here you would handle messages, e.g., by updating GUI elements or triggering ROS actions
                if message == 'start_ref':
                    self.ref_button_press()# TODO Change function call to refernce point mapping

        finally:
            # Schedule the next check of the queue
            self.root.after(100, self.check_queue)


    def solve_IK(self, x, z, initial_guesses, W, D, grid_size_x, grid_size_z):
        # Convert the (x, z) position to joint angles using IK

        solver_options = {
            'xtol': 1e-6,  # Adjust tolerance
            'maxfev': 10000  # Increase max function evaluations
        }

        # Solve the equations using fsolve
        solution = fsolve(equation_offset, initial_guesses, args=(x, z, D, W, grid_size_x, grid_size_z), full_output=True, **solver_options)
     
        # fsolve returns a tuple where the first element is the solution
        # and the fourth element is an integer flag indicating if a solution was found
        solution_values, infodict, ier, mesg = solution

        # Check the solution
        if ier != 1:
            self.get_logger().error(f'IK solution not found: {mesg}')
            return None, None, None, None

        # Extract the joint angles in radians from the solution
        theta1_left, theta2_left, theta1_right, theta2_right = solution_values

        """
        theta1_left: This is the angle of the joint connecting the left arm to the horizontal base. measured counterclockwise.

        theta2_left: This is the angle of the second joint of the left arm. It's the angle between the first arm segment (LL1) and the second segment (LL2), again measured counterclockwise. This angle describes the "elbow" bend of the left arm.

        theta1_right: This is the angle of the first joint of the right arm. Like theta1_left, it measures the orientation of the first segment (LR1) of the right arm relative to the horizontal base or reference line.

        theta2_right: This is the angle of the second joint of the right arm, the "elbow" angle. It measures how the second segment (LR2) of the right arm bends relative to the first segment.
        """
        # Calculate positions of each segment of the arm
        x_center = grid_size_x / 2
        z_base = grid_size_z
        x_base_left = x_center - W / 2
        x_base_right = x_center + W / 2

        # Left arm
        x_elbow_left = x_base_left + LL1 * np.cos(theta1_left)
        z_elbow_left = z_base + LL1 * np.sin(theta1_left)
        x_hand_left = x_elbow_left + LL2 * np.cos(theta1_left + theta2_left)
        z_hand_left = z_elbow_left + LL2 * np.sin(theta1_left + theta2_left)

        # Right arm
        x_elbow_right = x_base_right + LR1 * np.cos(theta1_right)
        z_elbow_right = z_base + LR1 * np.sin(theta1_right)
        x_hand_right = x_elbow_right + LR2 * np.cos(theta1_right + theta2_right)
        z_hand_right = z_elbow_right + LR2 * np.sin(theta1_right + theta2_right)

        arm_position_data = {
            "x_base_left": x_base_left, "z_base": z_base, "x_base_right": x_base_right,
            "x_left_elbow": x_elbow_left, "z_left_elbow": z_elbow_left,
            "x_right_elbow": x_elbow_right, "z_right_elbow": z_elbow_right,
            "x_left_hand": x_hand_left, "z_left_hand": z_hand_left,
            "x_right_hand": x_hand_right, "z_right_hand": z_hand_right
        }

        self.arm_position_pub.publish(String(data=json.dumps(arm_position_data)))
        return theta1_left, theta2_left, theta1_right, theta2_right

    
    def send_calculated_joint_angles(self, theta1_left, theta1_right):
        if self.mapping_done == False:
            # Create a message with the desired joint angles
            msg = Float64MultiArray()
            msg.data = [theta1_left, theta1_right]        
            
            # Publish the message to the controller topic
            # The topic and message type might be different for your setup
            self.joint_command_publisher.publish(msg)
            self.get_logger().info('Published calculated joint angles during mapping')

            return msg.data
        else:
            self.get_logger().info(f'---Mapping done---')
    
    def save_mappings_to_file(self):
        filename = 'robot_arm_mappings.json'
        """Saves the mapping to a JSON file."""
        with open(filename, 'w') as file:
            json.dump(self.mapping, file, indent=4)
        self.get_logger().info(f'Saved mappings to {filename}')
        print( "mapping saved to: "+ filename)

        
    
def main(args=None):
    rclpy.init(args=args)
    auto_mapper = AutoMapper()

    rclpy.spin(auto_mapper)
    rclpy.shutdown()


# Ensure the main function is called when the script is executed
if __name__ == '__main__':

    main()
