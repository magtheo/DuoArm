# display.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import matplotlib.pyplot as plt

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        self.subscription = self.create_subscription(
            String,
            'display_data',
            self.display_callback,
            10)
        self.subscription  # prevent unused variable warning

    def read_mapping(self, filename):
            with open(filename, 'r') as file:
                mapping = json.load(file)
            return mapping

    def display_callback(self, msg):
        data = json.loads(msg.data)
        self.visualize(data["mapping"], data["spine_points"], data["points"])

    def visualize(self):
        # Load mapping from the JSON file
        mapping = self.read_mapping('robot_arm_mappings.json')

        # Set up lists to collect coordinates
        inside_coords = {'x': [], 'z': []}
        outside_coords = {'x': [], 'z': []}

        # Iterate through the mapping and separate coordinates based on their status
        for key, value in mapping.items():
            x, z = map(int, key.split(','))  # Convert string to int
            status = value[2]  # 'inside' or 'outside'
            if status == 'inside':
                inside_coords['x'].append(x)
                inside_coords['z'].append(z)
            else:
                outside_coords['x'].append(x)
                outside_coords['z'].append(z)

        # Create the plot
        plt.figure(figsize=(10, 10))

        # Plot the points that are inside the workspace
        plt.scatter(inside_coords['x'], inside_coords['z'], color='green', label='Inside Workspace', alpha=0.6)

        # Plot the points that are outside the workspace
        plt.scatter(outside_coords['x'], outside_coords['z'], color='red', label='Outside Workspace', alpha=0.6)

        # Adding labels and legend
        plt.legend()
        plt.xlabel('X Coordinate')
        plt.ylabel('Z Coordinate')
        plt.title('Workspace Mapped with Coordinates')
        plt.grid(True)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    display_node = DisplayNode()

    display_node.visualize()
    rclpy.shutdown()

if __name__ == '__main__':
    main()