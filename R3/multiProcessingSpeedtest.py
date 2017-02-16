#from python docs

#
# Simple benchmarks for the multiprocessing package
#
# Copyright (c) 2006-2008, R Oudkerk
# All rights reserved.
#

import sys
import socket
import serial
import struct
import threading
import Queue
import time
import numpy
import random
import pyttsx
import math
import multiprocessing

from sys import platform

if platform == 'win32':
    _timer = time.clock
else:
    _timer = time.time

if platform == "linux" or platform == "linux2":
    import fcntl  # linux specific (keep note)
    import sys, getopt

    sys.path.append('.')
    import RTIMU #imu library
    import os.path

# -logging config
import logging
import logging.handlers
#the mask for data logging
logging.basicConfig(format='%(asctime)s - (%(threadName)s) %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p',
                    level=logging.DEBUG) #change the logging level here to change the display verbosity

delta = 1
exitFlag = multiprocessing.Value('b', False)

#global globalIMUData


#### TEST_QUEUESPEED
class cameraProcess(multiprocessing.Process):
    #def __init__(self, name):
    #     self.name = name
    #     #self.count = args[0]
    #    self._target = self.run()
    #     #self._args = tuple(args)
    #     self._name = name or type(self).__name__ + '-' + \
    #                          ':'.join(str(i) for i in self._identity)

    def run(self):
        #print self._args
        print self._args[1][0]
        print "printing 1 to count"
        for i in range(0, self._args[0]):
            print str(i)

        print self._args[1][1]
        self._args[1][1] = "im fine"

    def getVP(self):
        print "counting from 1 to 50"
        for i in range(0, 50):
            print str(i)

def writeThings (num, dataList):
    print dataList[0]
    print "printing 1 to count"
    for i in range(0, num):
        print str(i)

    print dataList[1]
    dataList.append("im fine")
    print dataList[2]

def IMU_Init(): #initialise IMU
    pass
    # global imu
    # global imuEnable


def trackPorterData(exitFlag, porterLocation_Global, porterOrientation):

    if platform == "linux" or platform == "linux2":  # if running on linux
        IMUSettingsFile = RTIMU.Settings("RTIMULib")  # load IMU caliberation file
        imu = RTIMU.RTIMU(IMUSettingsFile)  # initialise the IMU handle
        imuEnable = True  # IMU is initialised

    logging.info("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        logging.error("IMU Init Failed")
        # speechQueue.put("IMU Initiation Failed")
        imuEnable = False
    else:  # if connection successful...
        logging.info("IMU Init Succeeded")
        # speechQueue.put("IMU Successfully Initiated")
        imu.setSlerpPower(0.02)
        imu.setGyroEnable(True)
        imu.setAccelEnable(True)
        imu.setCompassEnable(True)
        imuEnable = True
        poll_interval = imu.IMUGetPollInterval()
        logging.debug("Recommended Poll Interval: %dmS\n" % poll_interval)
        logging.info("IMU is %s", str(imu))

    pulseData = ["",""] #pulses counted by the motor controller
    # Log
    logging.info("Trying to read IMU")
    logging.info("IMU is %s", str(imu))

    if imu.IMUInit():  # if the IMU is initiated...
        logging.info("IMU init successful")  # ...say so on the log
    else:
        logging.info("IMU init failed :/")  # ...or otherwise

    while not exitFlag.value:  # while the system isn't in shutdown mode
        if imu.IMURead():  # if data is available from the IMU...
            logging.debug("Reading IMU")
            globalIMUData = imu.getIMUData()  # read the IMU data into the global variable
            globalIMUFusion = globalIMUData["fusionPose"]  # extract AHRS fusion info
            time.sleep(imu.IMUGetPollInterval() * 1.0 / 1000.0)  # delay to sync with the recomended poll rate of the IMU

            porterOrientation.value = numpy.rad2deg(globalIMUFusion[2])  # get Yaw Data
            #print porterOrientation
            porterLocation_Global[0] = 10
            porterLocation_Global[1] = 15
        # print (str(pulsesQueue.empty()))
        # if not pulsesQueue.empty():  # if theres data on the pulse Queue...
        #     # logging.info("getting data")
        #     pulseData = pulsesQueue.get()  # ...get data
        #     # Convert to distance and do what needs to be done
        #     # print(str(self.pulseData))
        #     pulseData[0] = (pulseData[0][0]) + str(int("0x" + str(pulseData[0][1:]), 16))
        #     pulseData[1] = (pulseData[1][0]) + str(int("0x" + str(pulseData[1][1:]), 16))
        #
        #     porterLocation_Global, porterOrientation, wheelSpeeds = movePorter(pulseData[0], pulseData[1], porterLocation_Global,porterOrientation,wheelSpeeds)
        #
        #     pulsesQueue.task_done()  # set task complete in the Queue. (othewise the system will no be able to shut down)
        #


if __name__ == '__main__':
    # mgr = multiprocessing.Manager()
    # l = mgr.list(["hi", "how are you?"])
    # l[0] = "hola"
    #newProcess = cameraProcess(name="camProc", args=(50,l,))
    #newProcess.start()
    # newProc = multiprocessing.Process(target=writeThings, args=(50,l,))
    # newProc.start()

    mpManager = multiprocessing.Manager()
    globalIMUData = mpManager.dict()
    #globalIMUData = {None: None}
    testDict = mpManager.dict()
    globalIMUFusion = mpManager.list([.0, .0, .0])

    imuEnable = multiprocessing.Value('b', False)
    porterOrientation = multiprocessing.Value('d', 0.0)  # from north heading (in degrees?)
    porterLocation_Global = mpManager.list([0,0])


    imuProc = multiprocessing.Process(target=trackPorterData, args=(exitFlag, porterLocation_Global, porterOrientation,))
    imuProc.start()

    time.sleep(5)
    exitFlag.value = True
    imuProc.join()
    time.sleep(1)
    print porterLocation_Global[0]
    print testDict

    # while not exitFlag.value:
    #     print globalIMUFusion
    #     time.sleep(1)


    #time.sleep(2)

    #print l

    exitFlag = True