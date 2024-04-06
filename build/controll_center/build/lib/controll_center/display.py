# display.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')

        # Load the initial work area mapping from the JSON file
        self.mapping = self.read_mapping('robot_arm_mappings.json')

        self.subscription = self.create_subscription(
            String,
            'display_data',
            self.display_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.target_point_subscription = self.create_subscription(
            String,
            'target_point',
            self.target_point_callback,
            10)
        self.fig, self.ax = plt.subplots(figsize=(10, 10))

        # Initialize the plot elements with None, they will be created in animate()
        self.inside_scatter = None
        self.outside_scatter = None
        self.target_scatter = None


    def read_mapping(self, filename):
            with open(filename, 'r') as file:
                mapping = json.load(file)
            return mapping

    def target_point_callback(self, msg):
        self.target_point = json.loads(msg.data)
        # You may need to call a redraw/animation function here if using matplotlib interactive mode

    def display_callback(self, msg):
        data = json.loads(msg.data)
        self.visualize(data["mapping"], data["spine_points"], data["points"])

    def animate(self, i):
        # Redraw the plot with the updated target point
        # This assumes you've stored the axes objects as self.ax
        if self.target_point:
            self.ax.clear()
            # Replot the inside and outside points
            # Plot the target point
            self.ax.scatter(self.target_point['x'], self.target_point['z'], color='blue', label='Target Point', alpha=0.6)
            # Add other plot customizations here

    def animate(self, i):
        # Clear previous scatter to avoid overplotting
        if self.inside_scatter is not None:
            self.inside_scatter.remove()
        if self.outside_scatter is not None:
            self.outside_scatter.remove()
        if self.target_scatter is not None:
            self.target_scatter.remove()

        inside_coords = {'x': [], 'z': []}
        outside_coords = {'x': [], 'z': []}

        # Iterate through the mapping and separate coordinates based on their status
        for key, value in self.mapping.items():
            x, z = map(int, key.split(','))  # Convert string to int
            status = value[2]  # 'inside' or 'outside'
            if status == 'inside':
                inside_coords['x'].append(x)
                inside_coords['z'].append(z)
            else:
                outside_coords['x'].append(x)
                outside_coords['z'].append(z)

        # Update the inside and outside points
        self.inside_scatter = self.ax.scatter(inside_coords['x'], inside_coords['z'], color='green', label='Inside Workspace', alpha=0.6)
        self.outside_scatter = self.ax.scatter(outside_coords['x'], outside_coords['z'], color='red', label='Outside Workspace', alpha=0.6)

        # Plot the target point
        if self.target_point is not None:
            self.target_scatter = self.ax.scatter(self.target_point['x'], self.target_point['z'], color='blue', label='Target Point', alpha=0.6)

        # Re-draw the legend every time, so it updates the labels correctly
        self.ax.legend()
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Z Coordinate')
        self.ax.set_title('Workspace Mapped with Coordinates')
        self.ax.grid(True)

def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()

    # Create an animation that updates the plot
    ani = FuncAnimation(display_node.fig, display_node.animate, interval=1000)

    # Run the ROS node alongside the animation
    def ros_spin():
        rclpy.spin(display_node)

    # Use a separate thread to not block the matplotlib animation loop
    import threading
    spin_thread = threading.Thread(target=ros_spin)
    spin_thread.start()

    plt.show()  # This will block until the plot window is closed

    rclpy.shutdown()
    spin_thread.join()  # Ensure the ROS spinner thread exits cleanly

if __name__ == '__main__':
    main()