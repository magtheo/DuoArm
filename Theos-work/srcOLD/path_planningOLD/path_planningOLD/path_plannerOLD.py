
#!/usr/bin/env python3
# path_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState  # Missing import added
import json
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from .equation import equation_offset, D, W, grid_size_x, grid_size_z
from scipy.optimize import fsolve
from matplotlib.animation import FuncAnimation

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Publisher for display data
        self.display_publisher = self.create_publisher(String, 'display_data', 10)
 
        # publisher for joint angles
        self.joint_angles_publisher = self.create_publisher(Float64MultiArray, 'joint_angles_array', 10)

        # Topic for reading arm state from action controller
        self.state_subscription = self.create_subscription(
            String,
            'action_controller_state',
            self.state_callback,
            10)


    def start_path_processing(self):
        mapping = self.load_mapping('robot_arm_mappings.json')
        waypoints = self.generate_point_path()
        interpolated_points, points = self.interpolate(waypoints)
        joint_angles = self.map_path_to_angles(interpolated_points, mapping)
        self.joint_angles_publisher(joint_angles)
        self.publish_display_data(mapping, interpolated_points, points)

    def state_callback(self, msg):
        self.current_state = msg.data
        self.get_logger().info(f'path_planner current state: {self.current_state}')
        if self.current_state == 'path':
            self.get_logger().info('Starting path process...')
            self.start_path_processing()


    # Load the mapping from a JSON file
    def load_mapping(self, filename):
        with open(filename, 'r') as file:
            mapping = json.load(file)
        return mapping
    
    # Defining waypoints through which the path should go
    def generate_point_path(self):
        # Define waypoints as percentages of the grid dimensions
        relative_waypoints = np.array([
            [0.3, 0.1],
            [0.3, 0.4],
            [0.4, 0.6],
            [0.6, 0.6],
            [0.7, 0.5],
            [0.7, 0.4]   
        ])

        # Assume grid_size_x and grid_size_z are available as class variables or from a config
        waypoints = np.array([
            [wp[0] * grid_size_x, wp[1] * grid_size_z] for wp in relative_waypoints
        ])
        return waypoints

    def interpolate(self, waypoints):

        t = np.linspace(0, 1, len(waypoints))

        # Separate the x and z coordinates of the waypoints
        x_points = waypoints[:, 0]
        z_points = waypoints[:, 1]

        # Interpolate x and z as functions of t
        cs_x = CubicSpline(t, x_points)
        cs_z = CubicSpline(t, z_points)

    # Generate points along the spline curve for plotting or for use in path planning
        t_interp = np.linspace(0, 1, 500)
        x_interp = cs_x(t_interp)
        z_interp = cs_z(t_interp)
        spine_points = x_interp, z_interp
        return spine_points, (x_points, z_points)
    
    def solve_ik(self, x, z, initial_guesses):

        # Solve using fsolve
        solution = fsolve(equation_offset, initial_guesses, args=(x, z, D, W, grid_size_x, grid_size_z))
        return solution


    def map_path_to_angles(self, interpolated_points, mapping):
        joint_angles = []
        for x, z in zip(*interpolated_points):
            grid_x = int(x / grid_size_x * 100)
            grid_z = int(z / grid_size_z * 100)
            key = f"{grid_x},{grid_z}"

            if key in mapping and isinstance(mapping[key], list):
                angles = [np.radians(angle) for angle in mapping[key][:2]]
                initial_guesses = [angles[0], np.radians(90), angles[1], np.radians(270)]
            else:
                initial_guesses = [0, np.radians(90), 0, np.radians(270)]

            solution = self.solve_ik(x, z, initial_guesses)
            if not np.all(np.isfinite(solution)):
                self.get_logger().error(f"Invalid IK solution for point ({x}, {z}): {solution}")
                continue  # Skip this set of angles if the solution is not valid

            joint_angles.append(solution)

        joint_angles_array = np.array(joint_angles)
        if joint_angles_array.size == 0:
            self.get_logger().error("No valid joint angles generated.")
            return

        angles_msg = Float64MultiArray()
        angles_msg.data = joint_angles_array.flatten()
        self.joint_angles_publisher.publish(angles_msg)
        self.get_logger().info('Published joint angles to motor controller')
        return joint_angles_array


    def publish_display_data(self, mapping, spine_points, points):

        # Convert numpy arrays to lists
        spine_points_list = [spine_points[0].tolist(), spine_points[1].tolist()]
        points_list = [points[0].tolist(), points[1].tolist()]

        # Serialize data to JSON string for simplicity
        data = json.dumps({
            "mapping": mapping,
            "spine_points": spine_points_list,
            "points": points_list
        })
        msg = String()
        msg.data = data
        self.display_publisher.publish(msg)
        self.get_logger().info('Publishing display data')



        

def main(args=None):
    rclpy.init(args=args)

    path_planner =PathPlanner()

        
    rclpy.spin(path_planner)
    rclpy.shutdown()




if __name__ == '__main__':
    main()

