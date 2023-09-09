import math

class SimpleIMU:
    def __init__(self):
        # Filter parameters
        self.alpha = 0.98
        
        # Estimated orientation angles (pitch, roll, yaw)
        self.pitch = 0
        self.roll = 0
        
        # Gravity
        self.g = 9.81
    
    def update(self, ax, ay, az, gx, gy, gz, dt):
        # 1. Gyroscope angle estimation
        self.pitch += gx * dt
        self.roll += gy * dt

        # 2. Accelerometer angle estimation
        pitch_acc = math.atan2(ay, math.sqrt(ax**2 + az**2))
        roll_acc = math.atan2(-ax, az)
        
        # 3. Complementary filter
        self.pitch = self.pitch * self.alpha + pitch_acc * (1 - self.alpha)
        self.roll = self.roll * self.alpha + roll_acc * (1 - self.alpha)
        
        # 4. Gravity compensation
        gx_compensated = gx - self.g * math.sin(self.pitch)
        gy_compensated = gy + self.g * math.sin(self.roll) * math.cos(self.pitch)
        gz_compensated = gz + self.g * math.cos(self.roll) * math.cos(self.pitch)
        
        return gx_compensated, gy_compensated, gz_compensated

# Usage
imu = SimpleIMU()

# Sample data (replace this with your IMU readings)
ax, ay, az = 0.5, 0.5, 9.3  # accelerometer readings
gx, gy, gz = 0.01, -0.02, 0.03  # gyroscope readings

dt = 0.01  # time difference, you should adjust this based on your sampling rate

ax_new, ay_new, az_new = imu.update(ax, ay, az, gx, gy, gz, dt)
print(ax_new, ay_new, az_new)
