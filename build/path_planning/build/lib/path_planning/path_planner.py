
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
from .equation import equation, D, grid_size_x, grid_size_z # initial_guesses
from scipy.optimize import fsolve
from matplotlib.animation import FuncAnimation

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Publisher for display data
        self.display_publisher = self.create_publisher(String, 'display_data', 10)
 

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
            [1, 2],
            [3, 2],
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
    
    def solve_ik(self, x, z, initial_guesses):

        # Solve using fsolve
        solution = fsolve(equation, initial_guesses, args=(x, z, D))
        return solution

    def map_path_to_angles(self, interpolated_points):
        joint_angles = [self.solve_ik(x, z, initial_guesses) for x, z in zip(*interpolated_points)]
        return np.array(joint_angles)


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
    mapping = path_planner.load_mapping('robot_arm_mappings.json')
    
    waypoints = path_planner.generate_point_path()
    interpolated_points, points = path_planner.interpolate(waypoints)

    joint_angles = path_planner.map_path_to_angles(interpolated_points)
    # TODO Send joint angles to motor controller

    path_planner.publish_display_data(mapping, interpolated_points, points)
        
    rclpy.spin(path_planner)
    rclpy.shutdown()




if __name__ == '__main__':
    main()

