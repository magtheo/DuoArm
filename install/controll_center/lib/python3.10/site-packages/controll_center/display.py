# display.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
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
        self.data_ready = threading.Event()  # Event flag for data readiness
        # Create an animation that updates the plot
        self.ani = FuncAnimation(self.fig, self.update_display, interval=50)

        # Load the initial work area mapping from the JSON file
        # self.mapping = self.read_mapping('robot_arm_mappings.json')

        # Define a QoS profile for real-time updates
        real_time_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Initialize the plot elements with None, they will be created in update_display()
        self.inside_scatter = None
        self.outside_scatter = None
        self.target_scatter = None

        # Initialize variable to store the arm position
        self.arm_position = None
        self.arm_state = 'standby'  # Default state
        self.state_text = self.ax.text(0.5, 0.01, '', transform=self.ax.transAxes, ha='center')

        # Setup a timer to periodically call update_gui
        self.gui_update_timer = self.create_timer(0.1, self.update_display)  # every 100 ms

        # used to display predefined path
        self.subscription = self.create_subscription(
            String,
            'display_data',
            self.display_callback,
            10)

        # Add subscription to arm_position topic
        self.joint_angles_subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.joint_angles_callback,
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
        
        

    def display_callback(self, msg):
        data = json.loads(msg.data)
        with self.lock:
            self.process_new_data(data)
        self.request_redraw()



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
        theta1_left_rad = np.radians(theta1_left+180)
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
            pass

        self.state_text.set_text(f'State: {self.arm_state}')
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.grid(True)
        self.fig.canvas.draw_idle()




    def joint_angles_callback(self, msg):
        if not self.data_ready.is_set():# Check if data is not being processed
            self.data_ready.set() # Set the flag to block further processing
            try:
                self.get_logger().info(f"DISPLY recived arm position")
                data = msg.data
                theta1_left = data[0]
                theta1_right = data[1]
                self.compute_fk(theta1_left, theta1_right)
                self.request_redraw() 
            finally:
                self.data_ready.clear()# Clear the flag after processing

    def grid_data_callback(self, msg):
        grid_data = json.loads(msg.data)
        # Store the grid data for use in the update_display() or another visualization method
        self.grid_data = grid_data
        # You might want to trigger a redraw of the visualization here


    def state_callback(self, msg):
        self.arm_state = msg.data
        self.get_logger().info(f'Received new state: {self.arm_state}')

    def update_display(self):
        if self.data_ready.is_set():
            with self.lock: # lock when updating plot
                self.update_plot()
            self.data_ready.clear() # Reset the event flag after updating the plot



def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()

    # Use a separate thread to not block the matplotlib animation loop
    spin_thread = threading.Thread(target=lambda: rclpy.spin(display_node), daemon=True)
    spin_thread.start()
    plt.show()  # This will block until the plot window is closed

    rclpy.shutdown()
    spin_thread.join()  # Ensure the ROS spinner thread exits cleanly

if __name__ == '__main__':
    main()