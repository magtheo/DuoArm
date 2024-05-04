# display.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from rclpy.qos import QoSProfile, ReliabilityPolicy
import threading
import numpy as np




class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.lock = threading.Lock() # lock for thread-safe updates
        # Create an animation that updates the plot
        self.ani = FuncAnimation(self.fig, self.animate, interval=50)

        # Load the initial work area mapping from the JSON file
        # self.mapping = self.read_mapping('robot_arm_mappings.json')

        # Define a QoS profile for real-time updates
        real_time_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)

        # Initialize the plot elements with None, they will be created in animate()
        self.inside_scatter = None
        self.outside_scatter = None
        self.target_scatter = None

        # Initialize variable to store the arm position
        self.arm_position = None


        # used to display predefined path
        self.subscription = self.create_subscription(
            String,
            'display_data',
            self.display_callback,
            10)

        self.target_point_subscription = self.create_subscription(
            String,
            'target_point',
            self.target_point_callback,
            10)

        # Add subscription to arm_position topic
        self.arm_position_subscription = self.create_subscription(
            String,
            'arm_position',
            self.arm_position_callback,
            qos_profile=real_time_qos
        )

        self.grid_data_subscription = self.create_subscription(
            String,
            'grid_data',
            self.grid_data_callback,
            10)

        self.state_subscription = self.create_subscription(
            String,
            'action_controller_state',
            self.state_callback,
            10)
        
        self.arm_state = 'standby'  # Default state
        


    def read_mapping(self, filename):
        try:
            with open(filename, 'r') as file:
                mapping = json.load(file)
        except FileNotFoundError:
            self.get_logger().warn(f"File {filename} not found. Starting with empty mapping.")
            mapping = {}  # Initialize with an empty dictionary if the file is not found
        return mapping


    def target_point_callback(self, msg):
        self.target_point = json.loads(msg.data)
        # You may need to call a redraw/animation function here if using matplotlib interactive mode

    def display_callback(self, msg):
        data = json.loads(msg.data)
        with self.lock:
            self.process_new_data(data)
        self.request_redraw()

        # self.visualize(data["mapping"], data["spine_points"], data["points"])
        # self.update_plot()


    def compute_fk(self, theta1_left, theta1_right):
        # Constants for arm segments and geometry
        W = 20  # width between joints
        LL1 = 12  # length of upper left arm segment
        LR1 = 12  # length of upper right arm segment
        LL2 = 29  # length of lower left arm segment (forearm)
        LR2 = 29  # length of lower right arm segment (forearm)
        tool_length = 4 # length of the tool/hand offset
        grid_size_x = 100  # grid width
        grid_size_z = 100  # grid height

        # Compute base positions
        x_center = grid_size_x / 2
        z_base = grid_size_z
        x_base_left = x_center - W / 2
        x_base_right = x_center + W / 2

        # Convert angles from degrees to radians
        theta1_left_rad = np.radians(theta1_left)
        theta1_right_rad = np.radians(theta1_right)

        # Calculate positions of the elbows
        x_elbow_left = x_base_left + LL1 * np.cos(theta1_left_rad)
        z_elbow_left = z_base - LL1 * np.sin(theta1_left_rad)
        x_elbow_right = x_base_right + LR1 * np.cos(theta1_right_rad)
        z_elbow_right = z_base - LR1 * np.sin(theta1_right_rad)

        # Calculate positions of the hands (assuming the forearms are straight extensions)
        x_hand_left = x_elbow_left + LL2 * np.cos(theta1_left_rad)
        z_hand_left = z_elbow_left - LL2 * np.sin(theta1_left_rad)
        x_hand_right = x_elbow_right + LR2 * np.cos(theta1_right_rad)
        z_hand_right = z_elbow_right - LR2 * np.sin(theta1_right_rad)

        # Extend the forearms with the tool_length to keep the tool leveled
        x_tool_left = x_hand_left + tool_length  # Tool extends horizontally
        z_tool_left = z_hand_left  # Maintain level height
        x_tool_right = x_hand_right + tool_length  # Tool extends horizontally
        z_tool_right = z_hand_right  # Maintain level height

        # Store computed positions for visualization
        self.arm_position = {
            'x_base_left': x_base_left, 'z_base': z_base,
            'x_base_right': x_base_right, 'z_base': z_base,
            'x_elbow_left': x_elbow_left, 'z_elbow_left': z_elbow_left,
            'x_elbow_right': x_elbow_right, 'z_elbow_right': z_elbow_right,
            'x_hand_left': x_hand_left, 'z_hand_left': z_hand_left,
            'x_hand_right': x_hand_right, 'z_hand_right': z_hand_right,
            'x_tool_left': x_tool_left, 'z_tool_left': z_tool_left,
            'x_tool_right': x_tool_right, 'z_tool_right': z_tool_right
        }
        self.update_plot()


    def request_redraw(self):
        """Request an immediate redraw of the plot."""
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()


    def update_plot(self):
        self.ax.clear()
        if self.arm_position:
            # Draw base and arm segments
            self.ax.plot([self.arm_position['x_base_left'], self.arm_position['x_elbow_left']],
                        [self.arm_position['z_base'], self.arm_position['z_elbow_left']], 'ro-')
            self.ax.plot([self.arm_position['x_base_right'], self.arm_position['x_elbow_right']],
                        [self.arm_position['z_base'], self.arm_position['z_elbow_right']], 'bo-')
            self.ax.plot([self.arm_position['x_elbow_left'], self.arm_position['x_hand_left']],
                        [self.arm_position['z_elbow_left'], self.arm_position['z_hand_left']], 'r--')
            self.ax.plot([self.arm_position['x_elbow_right'], self.arm_position['x_hand_right']],
                        [self.arm_position['z_elbow_right'], self.arm_position['z_hand_right']], 'b--')
            self.ax.plot([self.arm_position['x_hand_left'], self.arm_position['x_tool_left']],
                     [self.arm_position['z_hand_left'], self.arm_position['z_tool_left']], 'g--', label='Left Tool'),
            self.ax.plot([self.arm_position['x_hand_right'], self.arm_position['x_tool_right']],
                     [self.arm_position['z_hand_right'], self.arm_position['z_tool_right']], 'g--', label='Right Tool')
    
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.grid(True)
        self.fig.canvas.draw_idle()



    # def update_plot(self):
    #     # Clear the plot to draw a new frame
    #     self.ax.clear() 

        # gammel kode
        # if self.arm_state == 'map':
        #     # Visualize received grid data with distinct styles for different point types
        #     if hasattr(self, 'grid_data'):
        #         for point in self.grid_data:
        #             x, z, point_type = point['x'], point['z'], point['type']
        #             if point_type == 'ref_top' or point_type == 'ref_bottom' or point_type == 'ref_max_x' or point_type == 'ref_min_x':
        #                 self.ax.scatter(x, z, color='yellow', s=100, edgecolor='black', label='Reference Point', alpha=0.8, zorder=5)
        

        # # Visualize robot arm base and end effectors
        # if self.arm_position:
        #     # Base positions
        #     self.ax.scatter([self.arm_position["x_base_left"], self.arm_position["x_base_right"]],
        #                     [self.arm_position["z_base"], self.arm_position["z_base"]],
        #                     color='blue', label='Base')

        #     # Arm segments: Shoulder to Elbow, Elbow to Hand
        #     self.ax.plot([self.arm_position["x_base_left"], self.arm_position["x_left_elbow"]],
        #                 [self.arm_position["z_base"], self.arm_position["z_left_elbow"]], 'r-')
        #     self.ax.plot([self.arm_position["x_left_elbow"], self.arm_position["x_left_hand"]],
        #                 [self.arm_position["z_left_elbow"], self.arm_position["z_left_hand"]], 'r--')

        #     self.ax.plot([self.arm_position["x_base_right"], self.arm_position["x_right_elbow"]],
        #                 [self.arm_position["z_base"], self.arm_position["z_right_elbow"]], 'b-')
        #     self.ax.plot([self.arm_position["x_right_elbow"], self.arm_position["x_right_hand"]],
        #                 [self.arm_position["z_right_elbow"], self.arm_position["z_right_hand"]], 'b--')

        #     self.ax.scatter([self.arm_position["x_left_hand"], self.arm_position["x_right_hand"]],
        #                     [self.arm_position["z_left_hand"], self.arm_position["z_right_hand"]],
        #                     color='green', label='Hands')

        # Clear previous scatters to avoid overplotting
        # scatters = [self.inside_scatter, self.outside_scatter, self.target_scatter]
        # for scatter in filter(None, scatters):  # Removes None from the list
        #     scatter.remove()

        # inside_coords = {'x': [], 'z': []}
        # outside_coords = {'x': [], 'z': []}

        # # Iterate through the mapping and separate coordinates based on their status
        # for key, value in self.mapping.items():
        #     x, z = map(int, key.split(','))  # Convert string to int
        #     status = value[2]  # 'inside' or 'outside'
        #     if status == 'inside':
        #         inside_coords['x'].append(x)
        #         inside_coords['z'].append(z)
        #     else:
        #         outside_coords['x'].append(x)
        #         outside_coords['z'].append(z)

        # # Update the inside and outside points
        # self.inside_scatter = self.ax.scatter(inside_coords['x'], inside_coords['z'], color='green', label='Inside Workspace', alpha=0.6)
        # self.outside_scatter = self.ax.scatter(outside_coords['x'], outside_coords['z'], color='red', label='Outside Workspace', alpha=0.6)

        # Plot the target point
        # if self.target_point is not None:
        #     self.target_scatter = self.ax.scatter(self.target_point['x'], self.target_point['z'], color='blue', label='Target Point', alpha=0.6)



        # # Re-draw the legend every time, so it updates the labels correctly
        # self.ax.legend()
        # self.ax.set_xlabel('X Coordinate')
        # self.ax.set_ylabel('Z Coordinate')
        # self.ax.set_title('Workspace Mapped with Coordinates')
        # self.ax.grid(True)

    def arm_position_callback(self, msg):
        self.get_logger().info(f"DISPLY recived arm position")
        data = json.loads(msg.data)
        theta1_left = data['theta1_left']
        theta1_right = data['theta1_right']
        self.compute_fk(theta1_left, theta1_right)

        # with self.lock:
        #     self.process_new_data(data)
        self.request_redraw() 

    def grid_data_callback(self, msg):
        grid_data = json.loads(msg.data)
        # Store the grid data for use in the animate() or another visualization method
        self.grid_data = grid_data
        # You might want to trigger a redraw of the visualization here


    def state_callback(self, msg):
        self.arm_state = msg.data
        self.get_logger().info(f'Received new state: {self.arm_state}')

    def animate(self, i):
        with self.lock: # lock when updating plot
            self.update_plot()

def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()

    # Create an animation that updates the plot
    # ani = FuncAnimation(display_node.fig, display_node.animate, interval=1000)

    # Run the ROS node alongside the animation
    def ros_spin():
        rclpy.spin(display_node)

    # Use a separate thread to not block the matplotlib animation loop
    spin_thread = threading.Thread(target=lambda: rclpy.spin(display_node), daemon=True)
    spin_thread.start()
    
    plt.show()  # This will block until the plot window is closed

    rclpy.shutdown()
    spin_thread.join()  # Ensure the ROS spinner thread exits cleanly

if __name__ == '__main__':
    main()