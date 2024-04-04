
#!/usr/bin/env python3
# path_planner.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState  # Missing import added
import json
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
from .equation import equation, D
from scipy.optimize import fsolve


class pathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
 

    # Load the mapping from a JSON file
    def load_mapping(self, filename):
        with open(filename, 'r') as file:
            mapping = json.load(file)
        return mapping
    
    # Defining waypoints through which the path should go
    # These are defined manually, but can be extracted from the mapping
    def generate_point_path(self):
        waypoints = np.array([
            [1, 1],
            [2, 1],
            [2, 2],
            [3, 2],
            [3, 3]
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
    
    def solve_ik(self, x, z):
        # Assuming initial guesses for the angles
        initial_guesses = np.radians([30, 90, -30, 90])
        # Solve using fsolve
        solution = fsolve(equation, initial_guesses, args=(x, z, D))
        return solution

    def map_path_to_angles(self, interpolated_points):
        joint_angles = [self.solve_ik(x, z) for x, z in zip(*interpolated_points)]
        return np.array(joint_angles)


    def visualize(self, mapping, spine_points, points):
        # Convert the mapping keys (string) back to x and z coordinates
        mapped_x, mapped_z = zip(*[map(int, key.split(',')) for key in mapping.keys()])
        
        # Extract interpolated points for plotting
        x_interp, z_interp = spine_points
        x_points, z_points = points

        # Create the plot
        plt.figure(figsize=(10, 6))
        
        # Plot the mapped points
        plt.scatter(mapped_x, mapped_z, color='red', label='Mapped Points', alpha=0.6)

        # Plot the waypoints
        plt.plot(x_points, z_points, 'o', label='Waypoints', color='yellow')
        
        # Plot the interpolated path
        plt.plot(x_interp, z_interp, label='Interpolated Path', color='blue')
        
        # Adding labels and legend
        plt.legend()
        plt.xlabel('X')
        plt.ylabel('Z')
        plt.title('Path Planning with Mapped Points and Interpolated Path')
        plt.grid(True)
        plt.show()




        

def main(args=None):
    rclpy.init(args=args)

    path_planner =pathPlanner()
    mapping = path_planner.load_mapping('robot_arm_mappings.json')
    
    waypoints = path_planner.generate_point_path()
    interpolated_points, points = path_planner.interpolate(waypoints)

    joint_angles = path_planner.map_path_to_angles(interpolated_points)

    path_planner.visualize(mapping, interpolated_points, points)




if __name__ == '__main__':
    main()

