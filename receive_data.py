import serial
import io

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
    adc_resolution = 2**15 - 1
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


while True:
    signal = Data.readline()
    signal = dataProcess(signal)
    print(signal)
