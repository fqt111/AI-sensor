import serial
import io
import math
import time
import numpy as np

Data = serial.Serial()
Data.baudrate = 38400
Data.port = "COM4"
Data.timeout = 0.01
Data.open()


def canFloat(data):
    try:
        float(data)
        return True
    except:
        return False


def adc_to_acceleration(adc_value, range=2):
    adc_resolution = 2**15
    voltage_from_adc = adc_value / adc_resolution
    acceleration = voltage_from_adc * range
    return acceleration


def dataProcess(data):
    data = str(data)
    dataSet = []
    datapoint = ""
    for i in data:
        if i.isdigit() or i == "-":
            datapoint += i
        elif i == "," and canFloat(datapoint):
            datapoint = adc_to_acceleration(float(datapoint), range=2)
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
    # print(magnitude)
    return np.abs(magnitude - 1) < 0.03


gravity = None
while True:
    # time.sleep(0.1)
    signal = Data.readline()
    signal = dataProcess(signal)

    # 单位化重力向量
    if len(signal) == 3:
        if stationary(signal):
            gravity = [signal[0], signal[1], signal[2]]
            print([0, 0, 0])
        elif gravity != None:
            corrected_data = correct_gravity(signal, gravity)
            print(corrected_data)
