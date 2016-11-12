import matplotlib.pyplot as plt
import serial
import numpy

hl, = plt.plot([0], [0])

# mainPlot, = plt.plot([0], [0], 'ro')
# plt.axis([0, 10, 0, 300])

plt.axis([0, 100, 0, 300])
plt.ion()
#plt.show()

# for i in range(10):
#     y = numpy.random.random()
#     plt.scatter(i, y)
#     plt.pause(0.05)
#
# while True:
#     plt.pause(0.05)
#
# def update_line(hl, new_data):
#
#     hl.set_xdata(numpy.append(hl.get_xdata(), new_data))
#     hl.set_ydata(numpy.append(hl.get_ydata(), new_data))
#     plt.draw()


try:
    usSensor = serial.Serial('COM4')
except Exception as e:
    print ('unable to establish serial comms')

i = 0

while True:
    usValue = int(usSensor.readline())
    print (usValue)
    plt.scatter(i, usValue)
    #plt.pause(0.01)

    #mainPlot.set_xdata(numpy.append(mainPlot.get_xdata(), int(usValue)))
    #plt.draw()
    #update_line(hl, int(usValue))

    i += 1
    if i > 300:
        i = 0
