import numpy as np


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
    roll = np.radians(roll)  # Convert degrees to radians
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)
    gravity_original = np.array([0, 0, -1])

    # Compute the rotation matrix
    R = rotation_matrix(roll, pitch, yaw)

    # Compute the gravity components in the rotated frame
    gravity_rotated = np.dot(R, gravity_original)

    return gravity_rotated


# Test
roll = np.radians(0)  # Convert degrees to radians
pitch = np.radians(-180)
yaw = np.radians(0)
gravity_rotated = gravity_components(roll, pitch, yaw)
res = 0
for i in gravity_rotated:
    res += i**2
print(res)
print(f"Gravity components in rotated frame: {gravity_rotated}")
