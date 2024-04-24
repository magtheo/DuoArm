import numpy as np
import unittest
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

# Define the lengths of the robot arm segments
LL1, LL2 = 12, 27  # Left arm segment lengths in cm
LR1, LR2 = 12, 27  # Right arm segment lengths in cm
W = 6              # Distance between the base joints in cm
D = 2              # Distance between the tool hub joints in cm
grid_size_x = 22
grid_size_z = 22

def equation(p, x_target, z_target, D, W, grid_size_x, grid_size_z):
    theta1_left, theta2_left, theta1_right, theta2_right = p
    
    x_center = grid_size_x / 2
    z_base = grid_size_z
    
    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    x_left = x_base_left + LL1 * np.cos(theta1_left) + LL2 * np.cos(theta1_left + theta2_left)
    z_left = z_base + LL1 * np.sin(theta1_left) + LL2 * np.sin(theta1_left + theta2_left)

    x_right = x_base_right + LR1 * np.cos(theta1_right) + LR2 * np.cos(theta1_right + theta2_right)
    z_right = z_base + LR1 * np.sin(theta1_right) + LR2 * np.sin(theta1_right + theta2_right)

    eq1 = (x_left + x_right) / 2 - x_target
    eq2 = (z_left + z_right) / 2 - z_target
    eq3 = z_left - z_right
    eq4 = np.sqrt((x_right - x_left)**2 + (z_right - z_left)**2) - D

    return (eq1, eq2, eq3, eq4)

def equationOLD( p, x_target, z_target, D):
    theta1_left, theta2_left, theta1_right, theta2_right = p

    # Calculate the position of the end effector for each arm
    x_left = LL1 * np.cos(theta1_left) + LL2 * np.cos(theta1_left + theta2_left)
    y_left = LL1 * np.sin(theta1_left) + LL2 * np.sin(theta1_left + theta2_left)

    x_right = W + LR1 * np.cos(theta1_right) + LR2 * np.cos(theta1_right + theta2_right)
    y_right = LR1 * np.sin(theta1_right) + LR2 * np.sin(theta1_right + theta2_right)

    # Equations describing the target position
    # Assuming the end-effector is at the midpoint between the left and right arms' ends
    eq1 = (x_left + x_right) / 2 - x_target
    eq2 = (y_left + y_right) / 2 - z_target

    # Equations to ensure the z-coordinates for the ends of both arms are the same
    eq3 = y_left - y_right

    # Constraint equation for the distance D between the tool hub joints
    eq4 = np.sqrt((x_right - x_left)**2 + (y_right - y_left)**2) - D

    return (eq1, eq2, eq3, eq4)

def solve_IK(x_target, z_target):
    initial_guesses = [np.radians(0), np.radians(90), np.radians(0), np.radians(90)]
    # solution = fsolve(equation, initial_guesses, args=(x_target, z_target, D, W, grid_size_x, grid_size_z))
    solution = fsolve(equationOLD, initial_guesses, args=(x_target, z_target, D))

    return solution

class TestRobotArmIK(unittest.TestCase):
    def test_target_at_center(self):
        x_target = grid_size_x / 2
        z_target = grid_size_z / 2
        theta1_left, theta2_left, theta1_right, theta2_right = solve_IK(x_target, z_target)
        self.assertIsNotNone(theta1_left)
        self.assertIsNotNone(theta1_right)

    def test_target_out_of_reach(self):
        x_target = grid_size_x * 2  # Unrealistically far
        z_target = grid_size_z * 2
        theta1_left, theta2_left, theta1_right, theta2_right = solve_IK(x_target, z_target)
        self.assertTrue(np.isnan(theta1_left) or np.isnan(theta1_right))


def plot_robot_arm(theta1_left, theta2_left, theta1_right, theta2_right, x_target, z_target):
    x_center = grid_size_x / 2
    z_base = grid_size_z

    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Calculate elbow positions based on first segment angles
    x_elbow_left = x_base_left + LL1 * np.cos(theta1_left)
    z_elbow_left = z_base + LL1 * np.sin(theta1_left)

    x_elbow_right = x_base_right + LR1 * np.cos(theta1_right)
    z_elbow_right = z_base + LR1 * np.sin(theta1_right)

    # Calculate end effector (hand) positions based on full arm angles
    x_hand_left = x_elbow_left + LL2 * np.cos(theta1_left + theta2_left)
    z_hand_left = z_elbow_left + LL2 * np.sin(theta1_left + theta2_left)

    x_hand_right = x_elbow_right + LR2 * np.cos(theta1_right + theta2_right)
    z_hand_right = z_elbow_right + LR2 * np.sin(theta1_right + theta2_right)

    plt.figure(figsize=(10, 8))

    # Plotting the arm segments: shoulder to elbow (upper arm) and elbow to hand (forearm)
    plt.plot([x_base_left, x_elbow_left], [z_base, z_elbow_left], 'r-', label='Left Upper Arm')
    plt.plot([x_elbow_left, x_hand_left], [z_elbow_left, z_hand_left], 'r--', label='Left Forearm')
    plt.plot([x_base_right, x_elbow_right], [z_base, z_elbow_right], 'b-', label='Right Upper Arm')
    plt.plot([x_elbow_right, x_hand_right], [z_elbow_right, z_hand_right], 'b--', label='Right Forearm')

    # Plotting the joints: base, elbow, and hand
    plt.plot(x_base_left, z_base, 'ko', label='Left Shoulder')
    plt.plot(x_base_right, z_base, 'ko', label='Right Shoulder')
    plt.plot(x_elbow_left, z_elbow_left, 'ro+', label='Left Elbow')
    plt.plot(x_elbow_right, z_elbow_right, 'bo+', label='Right Elbow')
    plt.plot(x_hand_left, z_hand_left, 'ro', label='Left Hand')
    plt.plot(x_hand_right, z_hand_right, 'bo', label='Right Hand')

    # Plotting the target point
    plt.plot(x_target, z_target, 'gx', markersize=10, label='Target')

    # Setting the limits, labels, and grid
    plt.xlim(0, grid_size_x)
    plt.ylim(0, grid_size_z)
    plt.title("Robot Arm Position Visualization")
    plt.xlabel("X position (cm)")
    plt.ylabel("Z position (cm)")
    plt.legend(loc='upper left')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    # Run unit tests
    unittest.main(exit=False)

    # Visualize the IK for the center target
    center_thetas = solve_IK(grid_size_x / 2, grid_size_z / 2)
    plot_robot_arm(*center_thetas, grid_size_x / 2, grid_size_z / 2)
