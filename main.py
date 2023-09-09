import serial

Data = serial.Serial()
Data.baudrate = 38400
Data.port = "COM5"
Data.timeout = 0.01
Data.open()
while True:
    signal = Data.readline()
    print(signal)
