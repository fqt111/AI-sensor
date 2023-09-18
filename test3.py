import serial
import io
import math
import time
import numpy as np
import angle_to_acc

Data = serial.Serial()
Data.baudrate = 38400
Data.port = "COM4"
Data.timeout = 0.1
Data.open()


def canFloat(data):
    try:
        float(data)
        return True
    except:
        return False


def adc_to_acceleration(adc_value, range=2):
    adc_resolution = 2**15
    value_list = []
    for value in adc_value:
        voltage_from_adc = value / adc_resolution
        acceleration = voltage_from_adc * range
        value_list.append(acceleration)
    return value_list


def adc_to_GRY(adc_value, range=1000):
    adc_resolution = 2**15
    value_list = []
    for value in adc_value:
        voltage_from_adc = value / adc_resolution
        acceleration = voltage_from_adc * range
        value_list.append(acceleration)
    return value_list


def dataProcess(data, convert_function=None):
    data = str(data)
    dataSet = []
    datapoint = ""
    for i in data:
        if i.isdigit() or i == "-":
            datapoint += i
        elif i == "," and canFloat(datapoint):
            # datapoint = convert_function(float(datapoint), range=2)
            dataSet.append(float(datapoint))
            datapoint = ""
    return dataSet


def correct_gravity(accel_data, gravity):
    corrected_data = (
        accel_data[0] - gravity[0],
        accel_data[1] - gravity[1],
        accel_data[2] - gravity[2],
    )

    return corrected_data


def stationary(accel_data):
    magnitude = math.sqrt(accel_data[0] ** 2 + accel_data[1] ** 2 + accel_data[2] ** 2)
    print("magnitude", magnitude)
    return np.abs(magnitude - 1) < 0.05


def calculate_roll_pitch_yaw(acceleration):
    x = acceleration[0]
    y = acceleration[1]
    z = acceleration[2]
    pitch = math.atan2(x, math.sqrt(y**2 + z**2)) * 180 / math.pi
    roll = math.atan2(y, math.sqrt(x**2 + z**2)) * 180 / math.pi
    yaw = 0
    return roll, pitch, yaw


class calculate_displacement(object):
    def __init__(self):
        self.displacement_x = 0
        self.displacement_y = 0
        self.displacement_z = 0

    def compute_displacement(self, gravity, sample_time=0.01):
        # Integration for displacement
        self.displacement_x += 9.81 * gravity[0] * (sample_time**2) * (10**3) / 2
        self.displacement_y += 9.81 * gravity[1] * (sample_time**2) * (10**3) / 2
        self.displacement_z += 9.81 * gravity[2] * (sample_time**2) * (10**3) / 2


def complementary_filter(roll, pitch, yaw, angle_pitch, angle_roll, angle_yaw, gry_data, time=0.001, alpha=0.98):
    angle_roll += gry_data[0] * time
    angle_roll += gry_data[1] * time
    yaw += gry_data[2] * time
    angle_pitch = angle_pitch * alpha + pitch * (1 - alpha)
    angle_roll = angle_roll * alpha + roll * (1 - alpha)
    angle_yaw = angle_yaw * alpha + yaw * (1 - alpha)
    return angle_roll, angle_pitch, angle_yaw


class KalmanFilterAngle:
    def __init__(self):
        # 卡尔曼滤波器变量初始化
        self.angle = 0  # 初始角度
        self.bias = 0  # 估计偏差
        self.P = np.array([[0, 0], [0, 0]])  # 卡尔曼滤波器协方差矩阵
        self.Q_angle = 0.001  # 角度的噪声协方差
        self.Q_bias = 0.003  # 偏差的噪声协方差
        self.R_measure = 0.03  # 测量噪声协方差

    def update(self, measured_angle, measured_angular_velocity, dt):
        # 用角速度预测下一个角度
        rate = measured_angular_velocity - self.bias
        self.angle += dt * rate

        # 更新估计协方差
        self.P[0][0] += dt * (dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # 计算卡尔曼增益
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]

        # 根据测量值更新估计值
        y = measured_angle - self.angle
        self.angle += K[0] * y
        self.bias += K[1] * y

        # 更新协方差
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle, rate


# 使用示例
kf1 = KalmanFilterAngle()
kf2 = KalmanFilterAngle()
kf3 = KalmanFilterAngle()
cd = calculate_displacement()
# angle_values = []  # 示例数据，代表测量到的角度
# angular_velocity_values = []  # 示例数据，代表测量到的角速度

# for i in range(len(angle_values)):
#     angle, rate = kf.update(angle_values[i], angular_velocity_values[i], 0.01)
#     # angle 是估计的角度，rate 是估计的角速度

count = 0
signal_list = []
averages = 0
while count != 100:
    signal = Data.readline()
    signal = dataProcess(signal)
    if len(signal) == 6:
        count += 1
        signal_list.append(signal)
        averages = [sum(col) / len(signal_list) for col in zip(*signal_list)]

# print("average", averages)
angle_roll = 0
angle_pitch = 0
angle_yaw = 0
count = 0
while True:
    # time.sleep(0.1)
    signal = Data.readline()
    signal = dataProcess(signal)
    # print(signal)
    signal[3] += 140
    signal[4] += 17
    signal[5] -= 22
    # print(signal)
    # print("raw_acc_data", signal[0:3])
    acc_data = adc_to_acceleration(signal[0:3])
    gry_data = adc_to_GRY(signal[3:6])
    acc_data[0] -= 0.087
    acc_data[2] += 0.01

    if len(signal) == 6:
        count += 1
        # if stationary(signal):
        #     gravity = [signal[0], signal[1], signal[2]]
        #     print([0, 0, 0, 0, 0, 0])
        # elif gravity != None:
        # print("acc_data", acc_data)
        # print("gry_data", gry_data)
        if count == 1:
            roll, pitch, yaw = calculate_roll_pitch_yaw(acc_data)
        # roll, pitch, yaw = complementary_filter(roll, pitch, yaw, angle_pitch, angle_roll, angle_yaw, gry_data)

        roll, roll_rate = kf1.update(roll, gry_data[0], 0.01)
        pitch, pitch_rate = kf2.update(pitch, gry_data[1], 0.01)
        yaw, yaw_rate = kf3.update(yaw, gry_data[2], 0.01)
        # print("roll, pitch, yaw ", roll, pitch, yaw)

        gravity_rotated = angle_to_acc.gravity_components(-roll, -pitch, yaw)
        # print(gravity_rotated)
        real_acc_data = []
        for i in range(3):
            real_acc_data.append(acc_data[i] + gravity_rotated[i])
        print(real_acc_data)
        # cd.compute_displacement(real_acc_data)
        # print(cd.displacement_x, cd.displacement_y, cd.displacement_z)
        # print("acc_data", real_acc_data)
