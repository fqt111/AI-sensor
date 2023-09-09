import math


def correct_gravity(accel_data):
    # Step 1: Compute the magnitude of the current acceleration vector
    magnitude = math.sqrt(accel_data[0] ** 2 + accel_data[1] ** 2 + accel_data[2] ** 2)

    # Step 2: Create a normalized gravity vector from the current acceleration data
    normalized_vector = (accel_data[0] / magnitude, accel_data[1] / magnitude, accel_data[2] / magnitude)

    # Step 3: Subtract this gravity vector from the current acceleration data
    corrected_data = (
        accel_data[0] - normalized_vector[0],
        accel_data[1] - normalized_vector[1],
        accel_data[2] - normalized_vector[2],
    )

    return corrected_data


# Test
accel_data = (1, 0, 0)  # just an example, replace with your actual IMU data
corrected_data = correct_gravity(accel_data)
print(corrected_data)
