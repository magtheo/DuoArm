#!/usr/bin/env python3
# equation.py

import numpy as np

# Define the lengths of the robot arm segments
LL1, LL2 = 20, 30  # Left arm segment lengths in cm
LR1, LR2 = 20, 30  # Right arm segment lengths in cm
W = 20             # Distance between the base joints in cm
D = 10             # Distance between the tool hub joints in cm

initial_guesses = np.radians([30, 90, -30, 90])

grid_size_x = 10;
grid_size_z = 10;


def equation( p, x_target, z_target, D):
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