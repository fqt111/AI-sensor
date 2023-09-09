import serial
import pyqtgraph as pg
import array
import numpy as np
import math
Data = serial.Serial()
Data.baudrate = 38400 # set baudrate
Data.port = 'COM3' # set port numberData.timeout = .01'
Data.open() # open the port
app = pg.mkQApp()
win = pg.GraphicsLayoutWidget()
win.setWindowTitle(' demo')
win.resize(1600,900)
xLength = 300

fig1 = win.addPlot()
fig1.showGrid(x=True,y=True)
fig1.setRange(xRange=[0,xLength],padding=0)
fig1.setLabel(axis='left', text='g')
fig1.setLabel(axis='bottom', text='x / point')
fig1.setTitle('acceleration')
curve1 = fig1.plot()
curve2 = fig1.plot()
curve3 = fig1.plot()
data = [np.zeros(xLength).__array__('d'),
np.zeros(xLength) .__array__('d'),
np.zeros(xLength).__array__('d')]

def dataProcess(data):
    data = str(data)
    dataSet = []
    datapoint = ''
    for i in data:
        if i.isdigit() or i == '-':
            datapoint += i
        elif i == "," and canFloat(datapoint):
            dataSet.append(float(datapoint))
            datapoint = ''
    return dataset
ax=array.array('d')
ay=array.array('d')
az=array.array('d')

def plotData():
    global signal
    signal = Data.readline()
    signal = dataProcess(signal)
    if (len(signal) == 3):
        for i in range(len(data)):
            if len(data[i]) < xLength:
                data[i].append(signal[i])
            else:
                data[i][:-1] = data[i][1:]
                data[i][-1] = signal[i]

        ax.append(signal[0])
        ax.append(signal[1])
        ax.append(signal[2])

        curve1.setData(data[o], pen=pg.mkPen('g',width=3))
        curve2.setData(data[1], pen=pg.mkPen('r', width=3))
        curve3.setData(data[2], pen=pg.mkPen('b',width=3))

timer = pg.Qtcore.QTimer()
timer.timeout.connect(plotData)
win.show()
timer.start(1)
app.exec()

try:
    input("Press Enter to stop\n")
except (Exception,KeyboardInterrupt):
    pass

np.savetxt("x_acc.txt", ax)
np.savetxt("y_acc.txt", ay)
np.savetxt("z_acc.txt", az)
app.close()