import numpy as np
import math


def rotation_matrix_from_euler_angles(angles):
    roll, pitch, yaw = angles
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])

    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    # Combine the rotation matrices
    rotation_matrix = np.dot(Rz, np.dot(Ry, Rx))

    return rotation_matrix


def rotation_matrix(roll, pitch, yaw):
    """Generate the combined rotation matrix for the given angles."""
    # Roll (x-axis rotation)
    R_x = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

    # Pitch (y-axis rotation)
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])

    # Yaw (z-axis rotation)
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R


def gravity_components(roll, pitch, yaw, g=9.81):
    """Calculate the gravity components in the rotated frame."""
    # Original gravity vector in the world frame
    # roll = np.radians(roll)  # Convert degrees to radians
    # pitch = np.radians(pitch)
    # yaw = np.radians(yaw)
    gravity_original = np.array([0, 0, -1])

    # Compute the rotation matrix
    R = rotation_matrix(roll, pitch, yaw)
    # rotation_matrix_from_euler_angles([roll, pitch, yaw])

    # Compute the gravity components in the rotated frame
    gravity_rotated = np.dot(gravity_original, R)

    return gravity_rotated


def calculate_roll_pitch_yaw(acceleration):
    x = acceleration[0]
    y = acceleration[1]
    z = acceleration[2]
    pitch = math.atan2(x, math.sqrt(y**2 + z**2)) * 180 / math.pi
    roll = math.atan2(y, z) * 180 / math.pi
    yaw = 0
    return roll, pitch, yaw


# Test
roll = np.radians(-67)  # Convert degrees to radians
pitch = np.radians(-34)
yaw = np.radians(0)
gravity_rotated = gravity_components(roll, pitch, yaw)
res = 0
for i in gravity_rotated:
    res += i**2
print(res)
print(f"Gravity components in rotated frame: {gravity_rotated}")

print(calculate_roll_pitch_yaw([0.57191304, -0.75701301, 0.31598541]))
