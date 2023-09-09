def remove_gravity(x, y, z, gravity_vector=(0, 0, 9.81)):
    """
    Subtract the gravity vector from the accelerometer readings.

    Parameters:
    - x, y, z: Accelerometer readings.
    - gravity_vector: A tuple representing the estimated gravity direction in the IMU's coordinate system.

    Returns:
    - x, y, z values with gravity removed.
    """
    x_without_gravity = x - gravity_vector[0]
    y_without_gravity = y - gravity_vector[1]
    z_without_gravity = z - gravity_vector[2]

    return x_without_gravity, y_without_gravity, z_without_gravity


# Example usage:
x, y, z = 0.5, -0.5, 10.31  # Sample accelerometer readings
x_new, y_new, z_new = remove_gravity(x, y, z)
print(x_new, y_new, z_new)
