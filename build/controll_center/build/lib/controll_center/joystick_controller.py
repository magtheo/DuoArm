# joystick_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import json

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        # Joystick subscription
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Publisher for target point data
        self.target_point_publisher = self.create_publisher(String, 'target_point', 10)

        # Load the work area from the JSON file
        self.work_area = self.load_work_area('robot_arm_mappings.json')

        # Initialize the target point in the middle of the work area
        self.target_point = self.initialize_target_point()



    def load_work_area(self, filename):
        with open(filename, 'r') as file:
            mapping = json.load(file)
        return mapping

    def initialize_target_point(self):
        # Find the center point that is inside the work area
        # You may need to customize this depending on how you define the center
        # For now, we'll just find the first 'inside' coordinate
        for key, value in self.work_area.items():
            if value[2] == 'inside':
                return {'x': int(key.split(',')[0]), 'z': int(key.split(',')[1])}


    # xbox controller
    def joy_callback(self, msg):
        # Define the dead zone threshold
        dead_zone_threshold = 0.5  # TODO Adjust and test 
        
        # Read joystick inputs
        horizontal_input = msg.axes[0]
        vertical_input = -msg.axes[1]  # Inverted

        # Apply dead zone
        if abs(horizontal_input) < dead_zone_threshold:
            horizontal_input = 0
        if abs(vertical_input) < dead_zone_threshold:
            vertical_input = 0

        # each joystick movement adjusts the target point
        scale_factor = 1  # TODO Adjust and test
        new_target_point = {
            'x': self.target_point['x'] + horizontal_input * scale_factor,
            'z': self.target_point['z'] + vertical_input * scale_factor,
        }

        # Check if the new target point is inside the work area
        if self.is_inside_work_area(new_target_point):
            self.target_point = new_target_point

            # Publish the updated target point
            target_point_msg = String()
            target_point_msg.data = json.dumps(self.target_point)
            self.target_point_publisher.publish(target_point_msg)
        else:
            # If the new target point is outside, log a message or handle accordingly
            self.get_logger().info(f"Attempted new target point {new_target_point} is outside the work area.")



    # HW-504 joystick
    # def joy_callback(self, msg):
    #     # Calculate the potential new target point based on joystick input
    #     new_target_point = {
    #         'x': self.target_point['x'] + int(msg.axes[0]),  # Horizontal axis, scale as needed
    #         'z': self.target_point['z'] + int(msg.axes[1]),  # Vertical axis, scale as needed
    #     }

    #     # Check if the new target point would be inside the work area
    #     if self.is_inside_work_area(new_target_point):
    #         # If inside, update the target point
    #         self.target_point = new_target_point

    #         # Publish the updated target point
    #         target_point_msg = String()
    #         target_point_msg.data = json.dumps(self.target_point)
    #         self.target_point_publisher.publish(target_point_msg)

    def is_inside_work_area(self, point):
        # Check if the given point is inside the work area
        key = f"{point['x']},{point['z']}"
        return key in self.work_area and self.work_area[key][2] == 'inside'




def main(args=None):
    rclpy.init(args=args)
    joystick_node = JoystickNode()
    rclpy.spin(joystick_node)
    joystick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
