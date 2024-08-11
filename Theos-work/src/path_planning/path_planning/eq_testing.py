import numpy as np
import unittest
from scipy.optimize import fsolve
import matplotlib.pyplot as plt

# Define the lengths of the robot arm segments
LL1, LL2 = 12, 29  # Left arm segment lengths in cm
LR1, LR2 = 12, 29  # Right arm segment lengths in cm
W = 6              # Distance between the base joints in cm
D = 2              # Distance between the tool hub joints in cm
grid_size_x = 40
grid_size_z = 40

min_thetaL_rad = np.radians(160)
max_thetaL_rad = np.radians(270)
min_thetaR_rad = np.radians(-90)
max_thetaR_rad = np.radians(40)

def equationOLD1(p, x_target, z_target, D, W, grid_size_x, grid_size_z):
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

def equationOLDButGOOD(p, x_target, z_target, D, W, grid_size_x, grid_size_z):
    theta1_left, theta2_left, theta1_right, theta2_right = p
    
    # Base position calculations
    x_center = grid_size_x / 2
    z_base = grid_size_z  # Assuming z_base is where the robot's shoulder joints are
    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Left arm calculations
    x_elbow_left = x_base_left + LL1 * np.cos(theta1_left)
    z_elbow_left = z_base + LL1 * np.sin(theta1_left)
    x_hand_left = x_elbow_left + LL2 * np.cos(theta1_left + theta2_left)
    z_hand_left = z_elbow_left + LL2 * np.sin(theta1_left + theta2_left)

    # Right arm calculations
    x_elbow_right = x_base_right + LR1 * np.cos(theta1_right)
    z_elbow_right = z_base + LR1 * np.sin(theta1_right)
    x_hand_right = x_elbow_right + LR2 * np.cos(theta1_right + theta2_right)
    z_hand_right = z_elbow_right + LR2 * np.sin(theta1_right + theta2_right)

    # Target position equations
    eq1 = (x_hand_left + x_hand_right) / 2 - x_target
    eq2 = (z_hand_left + z_hand_right) / 2 - z_target

    # Z-coordinate equality constraint for hands
    eq3 = z_hand_left - z_hand_right

    # Distance constraint between hands
    eq4 = np.sqrt((x_hand_right - x_hand_left)**2 + (z_hand_right - z_hand_left)**2) - D

    # Additional constraint to prevent hands from crossing
    # This forces the x-coordinate of the left hand to be always less than the right hand
    if x_hand_left >= x_hand_right:
        eq1 += 1000 * (x_hand_left - x_hand_right)  # Large penalty if the left hand crosses to the right side

    return (eq1, eq2, eq3, eq4)

def equation_center_both_hands(p, x_target, z_target, D, W, grid_size_x, grid_size_z):
    theta1_left, theta2_left, theta1_right, theta2_right = p
    
    # Base position calculations
    x_center = grid_size_x / 2
    z_base = grid_size_z
    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Left arm calculations
    x_elbow_left = x_base_left + LL1 * np.cos(theta1_left)
    z_elbow_left = z_base + LL1 * np.sin(theta1_left)
    x_hand_left = x_elbow_left + LL2 * np.cos(theta1_left + theta2_left)
    z_hand_left = z_elbow_left + LL2 * np.sin(theta1_left + theta2_left)

    # Right arm calculations
    x_elbow_right = x_base_right + LR1 * np.cos(theta1_right)
    z_elbow_right = z_base + LR1 * np.sin(theta1_right)
    x_hand_right = x_elbow_right + LR2 * np.cos(theta1_right + theta2_right)
    z_hand_right = z_elbow_right + LR2 * np.sin(theta1_right + theta2_right)

    # Modify the target position equations to ensure both hands meet at the exact target point
    eq1 = x_hand_left - x_target  # Left hand x-coordinate should be at the target
    eq2 = z_hand_left - z_target  # Left hand z-coordinate should be at the target
    eq3 = x_hand_right - x_target # Right hand x-coordinate should be at the target
    eq4 = z_hand_right - z_target # Right hand z-coordinate should be at the target

    return (eq1, eq2, eq3, eq4)

def equation_offset(p, x_target, z_target, D, W, grid_size_x, grid_size_z, offset=1):
    theta1_left, theta2_left, theta1_right, theta2_right = p
    
    # Base position calculations
    x_center = grid_size_x / 2
    z_base = grid_size_z
    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Left arm calculations
    x_elbow_left = x_base_left + LL1 * np.cos(theta1_left)
    z_elbow_left = z_base + LL1 * np.sin(theta1_left)
    x_hand_left = x_elbow_left + LL2 * np.cos(theta1_left + theta2_left)
    z_hand_left = z_elbow_left + LL2 * np.sin(theta1_left + theta2_left)

    # Right arm calculations
    x_elbow_right = x_base_right + LR1 * np.cos(theta1_right)
    z_elbow_right = z_base + LR1 * np.sin(theta1_right)
    x_hand_right = x_elbow_right + LR2 * np.cos(theta1_right + theta2_right)
    z_hand_right = z_elbow_right + LR2 * np.sin(theta1_right + theta2_right)

    # Hands positioned slightly left and right of the target
    eq1 = x_hand_left - (x_target - offset)  # Left hand x-coordinate offset to the left of target
    eq2 = z_hand_left - z_target  # Left hand z-coordinate to be at target height
    eq3 = x_hand_right - (x_target + offset) # Right hand x-coordinate offset to the right of target
    eq4 = z_hand_right - z_target  # Right hand z-coordinate to be at target height

    return (eq1, eq2, eq3, eq4)


def solve_IK(x_target, z_target):
    
    def constrained_equation(p):
        # Apply joint limits
        # p[0] = np.clip(p[0], min_thetaL_rad, max_thetaL_rad)  # Theta1 left
        # p[1] = np.clip(p[1], np.radians(-360), np.radians(360))  # Theta2 left, no specific limits given
        # p[2] = np.clip(p[2], min_thetaR_rad, max_thetaR_rad)  # Theta1 right
        # p[3] = np.clip(p[3], np.radians(-360), np.radians(360))  # Theta2 right, no specific limits given

        # Calculate the outputs of the IK equations
        errors = equation_offset(p, x_target, z_target, D, W, grid_size_x, grid_size_z)

        # Ensure the height difference between the two hands is zero (or close to it)
        # This could be your eq3 but ensuring we're returning the right format:
        return np.array(errors)  # Make sure it's an array of shape (4,)

    def constrained_equationOLD(p):
        # Define weights for each type of error
        weight_horizontal_alignment = 1  # Increase or decrease based on priority
        weight_position_error = 1  # Standard weight for position targeting

        # Apply joint limits
        # p[0] = np.clip(p[0], min_thetaL_rad, max_thetaL_rad)  # Theta1 left
        # p[1] = np.clip(p[1], np.radians(-360), np.radians(360))  # Theta2 left, no specific limits given
        # p[2] = np.clip(p[2], min_thetaR_rad, max_thetaR_rad)  # Theta1 right
        # p[3] = np.clip(p[3], np.radians(-360), np.radians(360))  # Theta2 right, no specific limits given


        # Get the error calculations from the main IK equation function
        eq1, eq2, eq3, eq4 = equation(p, x_target, z_target, D, W, grid_size_x, grid_size_z)

        # Prioritize horizontal alignment of the hands
        z_difference_error = (eq3 ** 2) * weight_horizontal_alignment

        # Adjust the importance of reaching the exact target position
        position_error_x = (eq1 ** 2) * weight_position_error
        position_error_z = (eq2 ** 2) * weight_position_error
        distance_error = (eq4 ** 2) * weight_position_error  # Ensuring the correct distance between hands

        return np.array([position_error_x, position_error_z, z_difference_error, distance_error])

    
    solver_options = {
        'xtol': 1e-6,  # Adjust tolerance
        'maxfev': 10000  # Increase max function evaluations
    }

    initial_guesses = [np.radians(220), np.radians(90), np.radians(340), np.radians(270)]
    # initial_guesses = [np.radians(180), np.radians(0), np.radians(0), np.radians(180)]
    solution = fsolve(constrained_equation, initial_guesses, xtol=1e-6, maxfev=10000)
    # solution = fsolve(equation, initial_guesses, args=(x_target, z_target, D, W, grid_size_x, grid_size_z), **solver_options)
    # solution = fsolve(equationOLD, initial_guesses, args=(x_target, z_target, D))

    return solution

class TestRobotArmIK(unittest.TestCase):
    def test_target_at_center(self):
        x_target = grid_size_x / 2
        z_target = grid_size_z / 2
        theta1_left, theta2_left, theta1_right, theta2_right = solve_IK(x_target, z_target)
        self.assertIsNotNone(theta1_left)
        self.assertIsNotNone(theta1_right)

    # def test_target_out_of_reach(self):
    #     x_target = grid_size_x * 2  # Unrealistically far
    #     z_target = grid_size_z * 2
    #     theta1_left, theta2_left, theta1_right, theta2_right = solve_IK(x_target, z_target)
    #     self.assertTrue(np.isnan(theta1_left) or np.isnan(theta1_right))

def plot_joint_limits(base_x, base_z, length, min_angle, max_angle, label, color):
    """Plot the joint limits as arcs."""
    angles = np.linspace(min_angle, max_angle, 100)
    x = base_x + length * np.cos(angles)
    z = base_z + length * np.sin(angles)
    plt.plot(x, z, color=color, linestyle='dotted', label=label)
    plt.show()

def plot_robot_armOLD(theta1_left, theta2_left, theta1_right, theta2_right, x_target, z_target):
    x_center = grid_size_x / 2
    z_base = grid_size_z

    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Convert radians to degrees for plotting
    theta1_left_deg = np.degrees(theta1_left)
    theta2_left_deg = np.degrees(theta1_left + theta2_left)
    theta1_right_deg = np.degrees(theta1_right)
    theta2_right_deg = np.degrees(theta1_right + theta2_right)

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

    # Plotting arm segments: shoulder to elbow (upper arm) and elbow to hand (forearm)
    plt.plot([x_base_left, x_elbow_left], [z_base, z_elbow_left], 'r-', label=f'Left Upper Arm ({theta1_left_deg:.1f}°)')
    plt.plot([x_elbow_left, x_hand_left], [z_elbow_left, z_hand_left], 'r--', label=f'Left Forearm ({theta2_left_deg:.1f}°)')
    plt.plot([x_base_right, x_elbow_right], [z_base, z_elbow_right], 'b-', label=f'Right Upper Arm ({theta1_right_deg:.1f}°)')
    plt.plot([x_elbow_right, x_hand_right], [z_elbow_right, z_hand_right], 'b--', label=f'Right Forearm ({theta2_right_deg:.1f}°)')

    # Plotting the joints: base, elbow, and hand
    plt.plot(x_base_left, z_base, 'ko', label='Left Shoulder')
    plt.plot(x_base_right, z_base, 'ko', label='Right Shoulder')
    plt.plot(x_elbow_left, z_elbow_left, 'r+', label='Left Elbow')
    plt.plot(x_elbow_right, z_elbow_right, 'b+', label='Right Elbow')
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

def plot_robot_arm(theta1_left, theta2_left, theta1_right, theta2_right, x_target, z_target):
    x_center = grid_size_x / 2
    z_base = grid_size_z

    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Convert radians to degrees for plotting
    theta1_left_deg = np.degrees(theta1_left)
    theta2_left_deg = np.degrees(theta1_left + theta2_left)
    theta1_right_deg = np.degrees(theta1_right)
    theta2_right_deg = np.degrees(theta1_right + theta2_right)

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

    # Plotting arm segments: shoulder to elbow (upper arm) and elbow to hand (forearm)
    plt.plot([x_base_left, x_elbow_left], [z_base, z_elbow_left], 'r-', label=f'Left Upper Arm ({theta1_left_deg:.1f}°)')
    plt.plot([x_elbow_left, x_hand_left], [z_elbow_left, z_hand_left], 'r--', label=f'Left Forearm ({theta2_left_deg:.1f}°)')
    plt.plot([x_base_right, x_elbow_right], [z_base, z_elbow_right], 'b-', label=f'Right Upper Arm ({theta1_right_deg:.1f}°)')
    plt.plot([x_elbow_right, x_hand_right], [z_elbow_right, z_hand_right], 'b--', label=f'Right Forearm ({theta2_right_deg:.1f}°)')

    # Plotting the joints: base, elbow, and hand with coordinates in the label
    plt.plot(x_base_left, z_base, 'ko', label=f'Left Shoulder ({x_base_left:.1f}, {z_base:.1f})')
    plt.plot(x_base_right, z_base, 'ko', label=f'Right Shoulder ({x_base_right:.1f}, {z_base:.1f})')
    plt.plot(x_elbow_left, z_elbow_left, 'r+', label=f'Left Elbow ({x_elbow_left:.1f}, {z_elbow_left:.1f})')
    plt.plot(x_elbow_right, z_elbow_right, 'b+', label=f'Right Elbow ({x_elbow_right:.1f}, {z_elbow_right:.1f})')
    plt.plot(x_hand_left, z_hand_left, 'ro', label=f'Left Hand ({x_hand_left:.1f}, {z_hand_left:.1f})')
    plt.plot(x_hand_right, z_hand_right, 'bo', label=f'Right Hand ({x_hand_right:.1f}, {z_hand_right:.1f})')

    # Plotting the target point with coordinates
    plt.plot(x_target, z_target, 'gx', markersize=10, label=f'Target ({x_target:.1f}, {z_target:.1f})')

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
    target_point_x = 20
    target_point_z = 18

    center_thetas = solve_IK(target_point_x, target_point_z)
    plot_robot_arm(*center_thetas, target_point_x, target_point_z)
