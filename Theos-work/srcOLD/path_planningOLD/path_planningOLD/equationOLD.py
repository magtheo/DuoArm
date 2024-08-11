#!/usr/bin/env python3
# equation.py

import numpy as np

# Define the lengths of the robot arm segments
LL1, LL2 = 12, 29  # Left arm segment lengths in cm
LR1, LR2 = 12, 29  # Right arm segment lengths in cm
W = 6             # Distance between the base joints in cm
D = 2             # Distance between the tool hub joints in cm

#initial_guesses = np.radians([30, 90, -30, 90])

grid_size_x = 40;
grid_size_z = 40;


def equationNotShifted( p, x_target, z_target, D):
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

def equation(p, x_target, z_target, D, W, grid_size_x, grid_size_z):
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

    return (eq1, eq2, eq3, eq4)

def equation_offset(p, x_target, z_target, D, W, grid_size_x, grid_size_z, offset=1):
    theta1_left, theta2_left, theta1_right, theta2_right = p
    adjusted_nullpoint_theta1_left = theta1_left + 180

    # Base position calculations
    x_center = grid_size_x / 2
    z_base = grid_size_z
    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Left arm calculations
    x_elbow_left = x_base_left + LL1 * np.cos(adjusted_nullpoint_theta1_left)
    z_elbow_left = z_base + LL1 * np.sin(adjusted_nullpoint_theta1_left)
    x_hand_left = x_elbow_left + LL2 * np.cos(adjusted_nullpoint_theta1_left + theta2_left)
    z_hand_left = z_elbow_left + LL2 * np.sin(adjusted_nullpoint_theta1_left + theta2_left)

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

def equationOld(p, x_target, z_target, D, W, grid_size_x, grid_size_z):
    theta1_left, theta2_left, theta1_right, theta2_right = p
    
    # Constants
    x_center = grid_size_x / 2  # X center of the grid
    z_base = grid_size_z  # Z base level

    # Adjust joint base positions for left and right arms
    x_base_left = x_center - W / 2
    x_base_right = x_center + W / 2

    # Calculate the position of the end effector for each arm
    x_left = x_base_left + LL1 * np.cos(theta1_left) + LL2 * np.cos(theta1_left + theta2_left)
    z_left = z_base + LL1 * np.sin(theta1_left) + LL2 * np.sin(theta1_left + theta2_left)

    x_right = x_base_right + LR1 * np.cos(theta1_right) + LR2 * np.cos(theta1_right + theta2_right)
    z_right = z_base + LR1 * np.sin(theta1_right) + LR2 * np.sin(theta1_right + theta2_right)

    # Equations describing the target position
    # Assuming the end-effector is at the midpoint between the left and right arms' ends
    eq1 = (x_left + x_right) / 2 - x_target
    eq2 = (z_left + z_right) / 2 - z_target

    # Equations to ensure the z-coordinates for the ends of both arms are the same
    eq3 = z_left - z_right

    # Constraint equation for the distance D between the tool hub joints
    eq4 = np.sqrt((x_right - x_left)**2 + (z_right - z_left)**2) - D

    return (eq1, eq2, eq3, eq4)


