import math


class GravityCompensator:
    def __init__(self):
        self.gravity_reference = [0, 0, 0]  # Initial reference value for gravity
        self.angle = [0, 0, 0]  # Initial angles for roll, pitch, yaw

    def calibrate(self, accel_data):
        # Set the initial reference for gravity using accelerometer data when the IMU is stationary
        self.gravity_reference = list(accel_data)

    def update_angles(self, gyro_data, dt):
        # Integrate the gyroscope data over time to estimate the angles
        for i in range(3):
            self.angle[i] += gyro_data[i] * dt

    def estimate_gravity_in_accel_frame(self):
        # Using trigonometry to rotate the gravity vector based on estimated angles
        # This is a simple model and may not be precise for larger rotations
        g = 9.81
        return (
            g * math.sin(self.angle[1]),
            -g * math.sin(self.angle[0]),
            g * math.cos(self.angle[0]) * math.cos(self.angle[1]),
        )

    def compensate_gravity(self, accel_data):
        estimated_gravity = self.estimate_gravity_in_accel_frame()
        return (
            accel_data[0] - estimated_gravity[0],
            accel_data[1] - estimated_gravity[1],
            accel_data[2] - estimated_gravity[2],
        )


# Usage:
compensator = GravityCompensator()

# Initially calibrate with IMU stationary
accel_data_initial = read_accel_data()  # Read from your IMU
compensator.calibrate(accel_data_initial)

while True:
    accel_data = read_accel_data()  # Read from your IMU
    gyro_data = read_gyro_data()  # Read from your IMU
    dt = 0.01  # Assuming a 10ms loop update rate, adjust accordingly

    compensator.update_angles(gyro_data, dt)
    corrected_data = compensator.compensate_gravity(accel_data)

    print(corrected_data)
    sleep(dt)
