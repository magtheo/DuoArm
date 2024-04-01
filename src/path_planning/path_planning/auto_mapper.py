
import numpy as np


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.optimize import fsolve


# Define the lengths of the robot arm segments
LL1, LL2 = 20, 30  # Left arm segment lengths in cm
LR1, LR2 = 30, 30  # Right arm segment lengths in cm
W = 20             # Distance between the base joints in cm
D = 10             # Distance between the tool hub joints in cm

class AutoMapper(Node):
    
    def __init__(self):
        super().__init__('auto_mapper')
        self.robot_arm = AutoMapper()
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.joint_state_msg = None
        self.mapping = {}  # To store the manually mapped coordinates with joint angles


    def map_workspace(self):
        for x in range(0, 1000, 10):  # Grid size
            for y in range(0, 1000, 10):

                # Command robot to move to the position (x, y)
                self.send_joint_angles(x, y)
                # Wait for robot to reach the position and stabilize
                self.wait_until_stable()
                # Record the joint states
                if self.joint_state_msg:
                    angle1, angle2 = self.read_joint_angles(self.joint_state_msg)
                    self.robot_arm.add_mapping(x, y, angle1, angle2)


    def joint_state_callback(self, msg):
        self.joint_state_msg = msg

    def send_joint_angles(self, x, y):
        # Convert the (x, y) position to joint angles using IK

        # Initial guesses for theta1 and theta2 for both arms
        initial_guesses = (0.5, 0.5, -0.5, -0.5)
        
        # Solve the equations using fsolve
        solution = fsolve(self.equations, initial_guesses, args=(x, y, D))
     

        # Check the solution
        if not solution:
            self.get_logger().error('IK solution not found')
            return

        # Extract the joint angles from the solution
        theta1_left, theta2_left, theta1_right, theta2_right = solution
     

        # Create a message with the desired joint angles
        msg = Float64MultiArray()
        msg.data = [theta1_left, theta1_right]        
        
        # Publish the message to the controller topic
        # The topic and message type might be different for your setup
        self.joint_command_publisher.publish(msg)
        self.get_logger().info('Published joint angles to move robot to position')
    

    def equations(self, p, x_target, y_target, D):
        theta1_left, theta2_left, theta1_right, theta2_right = p

        # Calculate the position of the end effector for each arm
        x_left = LL1 * np.cos(theta1_left) + LL2 * np.cos(theta1_left + theta2_left)
        y_left = LL1 * np.sin(theta1_left) + LL2 * np.sin(theta1_left + theta2_left)

        x_right = W + LR1 * np.cos(theta1_right) + LR2 * np.cos(theta1_right + theta2_right)
        y_right = LR1 * np.sin(theta1_right) + LR2 * np.sin(theta1_right + theta2_right)

        # Equations describing the target position
        # Assuming the end-effector is at the midpoint between the left and right arms' ends
        eq1 = (x_left + x_right) / 2 - x_target
        eq2 = (y_left + y_right) / 2 - y_target

        # Equations to ensure the y-coordinates for the ends of both arms are the same
        eq3 = y_left - y_right

        # Constraint equation for the distance D between the tool hub joints
        eq4 = np.sqrt((x_right - x_left)**2 + (y_right - y_left)**2) - D

        return (eq1, eq2, eq3, eq4)

    def read_joint_angles(self, joint_state_msg):
        # Extract joint angles from the joint_state_msg
        # This is simplified; you'd extract the specific joint angles you need
        angle1 = joint_state_msg.position[0]
        angle2 = joint_state_msg.position[1]
        return angle1, angle2
    
# Initialize ROS2 and the AutoMapper node
    
def main(args=None):
    rclpy.init(args=args)
    auto_mapper = AutoMapper()
    
    #Itterate over the coordinates and run send_joint_angles

    # Example target coordinates for the end-effector
    x_target = 15  # x position in cm
    y_target = 15  # y position in cm

    auto_mapper.map_workspace(x_target, y_target)




# Ensure the main function is called when the script is executed
if __name__ == '__main__':
    main()






