#
# Module name - R3_server
# Module Description -

# Author     - C. Samarakoon
# Created    - 16/02/2017
# Modified   - 22/02/2017
#
###---Imports-------------------
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
import glob
import datetime

from sys import platform
#if on linux, import functions for IMU and ip
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
                    level=logging.INFO) #change the logging level here to change the display verbosity

try:
    import vpLib
except Exception as e:
     logging.error("Cant import vpLib")

###---Global Variables-------------


##--Motor Commands
global lastCommand #holds the last command that was sent to the motor controllers
global speedVector #demanded wheel speeds (left, right)
global dataReady #boolean to let the threads know that data is ready to be sent to the motors
global wheelSpeeds #measured wheel speeds

lastCommand = ""
speedVector = [0, 0]
lastSent = [0, 0]

dataReady = False

# global USConnected
# motorConnected = False

##--Serial Connections
global MotorConn #serial handle for the motor controller
global USConn1 #serial handle for the Ultrasonic controller 1
global USConn2 #serial handle for the Ultrasonic controller 2

##--Safety
global safetyOn #Boolean for the state of the safety toggle (used only in manual control)
safetyOn = True

##--US data
global USAvgDistances #vector holding the average US distances
global obstruction #Boolean for comminicating whether there is an obstruction in the direction of travel
global UShosts

#F_bot, F_left, F_right, L_mid, R_mid, B_mid - F_top, L_front, R_front, L_back, R_back, B_left, B_right
USAvgDistances = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
obstruction = False
USThresholds = [50, 40, 40] #threasholds for treating objects as obstacles [front,side,back]
UShosts = 2

##--Multi-Threading/Muti-processing
global threadLock #lock to be used when changing global variables
global speechQueue #Queue holding sentences to be spoken
global pulsesQueue #Queue holding the measured wheel encoder pulses
global serverFeedbackQueue

threadLock = threading.Lock()
speechQueue = multiprocessing.Queue()
pulsesQueue = multiprocessing.Queue()
serverFeedbackQueue = multiprocessing.Queue()

threads = [] #Array holding information on the currently running threads
processes = [] #Array holding information on the currently running Processes

# -QR Codes
global QRdetected #Boolean for the QRcode detection status
global QRdata #String read from the QR code

QRdetected = False
QRdata = ""

# -Camera Functions

# -IMU data
#global imu #Handle for the IMU
global imuEnable #Boolean holding whether IMU is connected
imuEnable = multiprocessing.Value('b', False)

##--Auto Pilot
global autoPilot #Boolean for turning on/off autopilot
autoPilot = False #autopilot turned off

# -Porter Localisation

global porterLocation_Global #vector holding global location [x,y] in cm
global porterLocation_Local #vector holding local location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
global targetDestination  #Target location in global coordinates in cm
global distanceToGoal #Distance to goal in cm
global porterLocation_IMU
targetDestination = [0,0]

porterOrientation = multiprocessing.Value('d',0.0)  # from north heading (in degrees?)
distanceToGoal = 0

orientationIMU = multiprocessing.Value("d", 0.0)
orientationWheels = multiprocessing.Value("d", 0.0)

##-System State Variables
localCtrl = True #Local control = commands sent through SSH not TCP/IP
sysRunning = False #
cmdExpecting = False #
#exitFlag = False #set exitFlag = True initiate system shutdown
dataInput = "" #Data string from the user (SSH/TCP)

exitFlag = multiprocessing.Value('b', False) #multiprocessing Exit flag (can be combined with the others)

global motor_portAddr
global US1_portAddr
global US2_portAddr

motor_portAddr = ""
US1_portAddr = ""
US2_portAddr = ""

global h_scores
h_scores = [0.,0.,0.,0.]

def serialDetect():
#This fuction will ping all serial connections to find out what each device is

    global motor_portAddr
    global US1_portAddr
    global US2_portAddr

    if platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
        print "ports are " + ports
    elif platform == "linux" or platform == "linux2":
        #check this
        ports = glob.glob('/dev/tty[A-Za-z]*')
        print "ports are " + ports
    else:
        logging.error("Unsupported Platform")

    for port in ports:
        try:
            testConn = serial.Serial(port, 19200)
            logging.info('Checking device at ', port)
            testConn.write("whois?\n")
            response = testConn.readline()
            if response == "motor\n":
                logging.info('Motor Controller at ', port)
                motor_portAddr = port
            elif response == "us1\n":
                logging.info('US 1 at ', port)
                US1_portAddr = port
            elif response == "us2\n":
                logging.info('US 2 at ', port)
                US2_portAddr = port
        except Exception as e:
            logging.error("Error Checking Serial ports - %s", e)

def ConnectToSerial():
    global MotorConn
    global USConn1
    global USConn2

    serialDetect()

    logging.info("Trying to connect to Serial Devices")
    if motor_portAddr != "":
        try:
            MotorConn = serial.Serial(motor_portAddr, 19200)
        except Exception as e:
            logging.error("%s", e)

    if US1_portAddr != "":
        try:
            USConn1 = serial.Serial(US1_portAddr, 19200)
        except Exception as e:
            logging.error("%s", e)

    if US2_portAddr != "":
        try:
            USConn2 = serial.Serial(US2_portAddr, 19200)
        except Exception as e:
            logging.error("%s", e)

def setExitFlag(status):
    global exitFlag
    global exitFlag

    exitFlag = status
    exitFlag.value = status

###---Class Definitions
# create a class for the data to be sent over ait

class MultiThreadBase(threading.Thread): #Parent class for threading
    def __init__(self, threadID, name): #Class constructor
        threading.Thread.__init__(self) #Initiate thread
        self.threadID = threadID #Thread number
        self.name = name #Readable thread name

        #thread loop performance profiling
        self.avgRuntime = 0. #loop average runtime
        self.startTime = 0. #loop start time
        self.endTime = 0. #loop end time
        self.avgCounter = 0 #number of loops
        self.loopProfilingInterval = 10 #loop averaging interval
        self.profiling = False #profiling state. set true in to calculate run times globally.
            #if you want to profile a single thread, change this in the child thread
            #Note - the system will run much slower when profiling
            #STILL NOT FULLY IMPLEMENTED. DO NOT SET TO TRUE

    def loopStartFlag(self): #run at the start of the loop
        self.startTime = time.localtime() #record the start time of the loop

    def loopEndFlag(self):
        self.startTime = time.localtime()

    def loopRunTime(self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime) / self.loopProfilingInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopProfilingInterval:
            # print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            logging.debug("Avg loop Runtime - %s", str(self.avgRuntime))
            self.avgCounter = 0


##---------Mapping process


##----------IMU process

def porterTracker(exitFlag,imuEnable, porterLocation_Global, porterOrientation, wheelSpeeds, pulsesQueue, speechQueue):

    AHRSmode = "imu"  # can be imu or wheel. Defaults to wheel
    pulseData = ["", ""]  # pulses counted by the motor controller

    # if AHRSmode is not "imu" or "wheel":
    #     AHRSmode = "wheel"

    if platform == "linux" or platform == "linux2":  # if running on linux
        IMUSettingsFile = RTIMU.Settings("mainIMUcal")  # load IMU caliberation file
        imu = RTIMU.RTIMU(IMUSettingsFile)  # initialise the IMU handle
        imuEnable.value = True  # IMU is initialised

    if imuEnable.value:
        logging.info("IMU Name: " + imu.IMUName())

        if (not imu.IMUInit()):
            logging.error("IMU Init Failed")
            speechQueue.put("IMU Initiation Failed")
            imuEnable.value = False
        else:  # if connection successful...
            logging.info("IMU Init Succeeded")
            speechQueue.put("IMU Successfully Initiated")
            imu.setSlerpPower(0.02)
            imu.setGyroEnable(True)
            imu.setAccelEnable(True)
            imu.setCompassEnable(True)
            imuEnable.value = True
            poll_interval = imu.IMUGetPollInterval()
            logging.debug("Recommended Poll Interval: %dmS\n" % poll_interval)
            logging.info("IMU is %s", str(imu))

    while not exitFlag.value:  # while the system isn't in shutdown mode
        if imuEnable.value:
            if imu.IMURead():  # if data is available from the IMU...
                logging.debug("Reading IMU")
                globalIMUData = imu.getIMUData()  # read the IMU data into the global variable
                globalIMUFusion = globalIMUData["fusionPose"]  # extract AHRS fusion info NOTE
                time.sleep(imu.IMUGetPollInterval() * 1.0 / 1000.0)  # delay to sync with the recomended poll rate of the IMU
                if AHRSmode is "imu":
                    porterOrientation.value = globalIMUFusion[2]  # get Yaw Data

        if not pulsesQueue.empty():  # if theres data on the pulse Queue...
            #logging.info("getting data")
            pulseData = pulsesQueue.get()  # ...get data
            # Convert to distance and do what needs to be done
            print(str(pulseData))
            try:
                pulseData[0] = int((pulseData[0][0]) + str(int("0x" + str(pulseData[0][1:]), 16)))
                pulseData[1] = int((pulseData[1][0]) + str(int("0x" + str(pulseData[1][1:]), 16)))

                porterLocation_Global, wheelSpeeds, orientationWheels.value = movePorter(pulseData[0], pulseData[1], porterLocation_Global,porterOrientation,wheelSpeeds,orientationWheels)
                if AHRSmode is "wheel":
                    porterOrientation.value = orientationWheels.value

            except Exception as e:
                logging.error("%s", str(e))


def movePorter(lPulses, rPulses, porterLocation_Global, porterOrientation,wheelSpeeds,orientationWheels):  # function for calculating the distance moved
    # Assume that it doesnt "move" when rotating
    # Need to check if this function is mathematically sound for all r, theta.

    timeInterval = 0.5  # sampling interval of the motor controller
    wheelRadius = 20
    pulsesPerRev = 360
    wheelBaseWidth = 71
    # find the wheel speeds
    wheelSpeeds[0] = ((lPulses / timeInterval) * 60) / pulsesPerRev
    wheelSpeeds[1] = ((rPulses / timeInterval) * 60) / pulsesPerRev
    # print(str(wheelSpeeds))

    # find the distance moved
    lDist = (2 * numpy.pi * lPulses / pulsesPerRev) * wheelRadius
    rDist = (2 * numpy.pi * rPulses / pulsesPerRev) * wheelRadius

    r = (lDist + rDist) / 2  # take the average of the distances for now.
    distDelta = (lDist - rDist)

    orientationWheels.value += distDelta/wheelBaseWidth

    if orientationWheels.value > numpy.pi:
        orientationWheels.value -= 2*numpy.pi
    elif orientationWheels.value < -numpy.pi:
        orientationWheels.value += 2 * numpy.pi

        # # r is the staight line distance traveled... so find x,y components
    #if lastCommand == "f":

    porterLocation_Global[0] += r * numpy.cos(numpy.pi/2 - porterOrientation.value)  # x component
    porterLocation_Global[1] += r * numpy.sin(numpy.pi/2 - porterOrientation.value)  # y component

    # elif lastCommand == "b":
    #     porterLocation_Global[0] = porterLocation_Global[0] - r * numpy.cos(numpy.pi/2 - porterOrientation.value)  # x component
    #     porterLocation_Global[1] = porterLocation_Global[1] - r * numpy.sin(numpy.pi/2 - porterOrientation.value)  # y component

    #print (str(porterLocation_Global) + " __ " + str(porterOrientation.value)+ " __ " + str(orientationWheels.value))

    return porterLocation_Global,wheelSpeeds,orientationWheels.value


class autoPilotThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name

        self.angleToGoal = 0 #angle between the goal and the robot
        self.angleChange = 0 #angle change required

        self.dX = 0
        self.dY = 0

        #path-finding parameters
        self.recPenalty = 0.2 #penalty for recursion
        self.momentumBonus = 0.5 #bonus for momentum conservation
        self.alpha = 1 #
        self.beta = 10

        #Autopilot Control parameters
        self.looping = True
        self.obs = True
        self.distQuantise = 30  # path re-calculation distance
        self.autoSpeed = 10  # autopilot movement speed
        self.alignmentThreshold = 5  # alignment error in degrees
        self.hScores = []
        self.bestScoreIndex = 0

    def run(self):
        global threadLock
        global dataReady
        global speedVector
        global distanceToGoal
        global porterLocation_Global
        global porterLocation_Local
        global lastCommand
        global targetDestination
        global autoPilot
        global h_scores

        while not exitFlag.value: #while the system isn't in shutdown mode
            if not autoPilot: #if autopilot is disabled...

                targetDestination = [100,0]

                self.hScores = self.checkquad()
                h_scores = self.hScores

                time.sleep(1) #...sleep for a bit ...zzzzzzz
            elif not imuEnable.value:
                logging.warning("IMU not available. Can't turn on Autopilot... Soz...")
                speechQueue.put("IMU not available. Can't turn on Autopilot...")
                autoPilot = False
            elif self.looping and self.obs:
                porterLocation_Global = [0,0]  # for now assume local position is the same as global
                # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER
                logging.info("Iteratve Autopilot mode with Obstacle Avoidance")
                speechQueue.put("Iteratve Autopilot mode with Obstacle Avoidance")  # vocalise

                self.dX = targetDestination[0] - porterLocation_Global[0]
                self.dY = targetDestination[1] - porterLocation_Global[1]

                while numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY)) > 30 and not exitFlag.value:
                # find the angle of the goal from north

                    speechQueue.put("Quantising the environment")
                    logging.info("Quantising The environment")
                    time.sleep(2)

                    self.hScores = self.checkquad()
                    h_scores = self.hScores

                    if self.bestScoreIndex == 0:
                        speechQueue.put("Best way to go is forward")
                        logging.info("Best way to go is forward")
                        self.moveStraight(dist=30,direction="f")
                    elif self.bestScoreIndex == 1:
                        speechQueue.put("Best way to go is backwards")
                        logging.info("Best way to go is backwards")
                        self.moveStraight(dist=30, direction="b")
                    elif self.bestScoreIndex == 2:
                        speechQueue.put("Best way to go is left")
                        logging.info("Best way to go is left")
                        self.turn(angleChange= -90)
                    elif self.bestScoreIndex == 3:
                        speechQueue.put("Best way to go is right")
                        logging.info("Best way to go is right")
                        self.turn(angleChange= 90)

                    self.dX = targetDestination[0] - porterLocation_Global[0]
                    self.dY = targetDestination[1] - porterLocation_Global[1]

                    if obstruction:
                        time.sleep(5)
                        speechQueue.put("sleeping for a bit")

                speechQueue.put("Successfully arrived at the target")
                logging.info("Successfully arrived at the target")
                autoPilot = False  # turn off autopilot at the end of the maneuver

            elif self.looping: #iterative mode
                porterLocation_Global = porterLocation_Local  # for now assume local position is the same as global
                # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER
                logging.info("Iteratve Autopilot mode")
                speechQueue.put("Iteratve Autopilot mode")  # vocalise

                speechQueue.put("Looking for the target destination")
                logging.info("Looking for the target destination")

                self.dX = targetDestination[0] - porterLocation_Global[0]
                self.dY = targetDestination[1] - porterLocation_Global[1]

                while numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY)) > 30:
                # find the angle of the goal from north

                    self.dX = targetDestination[0] - porterLocation_Global[0]
                    self.dY = targetDestination[1] - porterLocation_Global[1]

                    if self.dX != 0:
                        self.angleToGoal = numpy.arctan(self.dY / self.dX)
                    else:
                        self.angleToGoal = numpy.pi / 2

                    self.angleToGoal = numpy.rad2deg(self.angleToGoal)

                    # convert angle to 0,2pi
                    if self.dX >= 0:  # positive x
                        if self.dY >= 0:
                            self.angleToGoal = numpy.rad2deg(porterOrientation.value) - self.angleToGoal
                        if self.dY < 0:  # negative y
                            self.angleToGoal = numpy.rad2deg(porterOrientation.value) - self.angleToGoal

                    elif self.dX < 0:  # negative x
                        if self.dY >= 0:  # positive y
                            self.angleToGoal = numpy.rad2deg(porterOrientation.value) - (180 + self.angleToGoal)
                        if self.dY < 0:  # negative y
                            self.angleToGoal = numpy.rad2deg(porterOrientation.value) - (180 + self.angleToGoal)

                    # at this point angleToGoal is defined relative to north
                    # if this works remove redundant IF statements. Currently used only for debugging.

                    # same as before, finding the angle that needs to be changed
                    self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                    if self.angleChange < -180:
                        self.angleChange += 360

                    if autoPilot and not exitFlag.value:
                        with threadLock:
                            if self.angleChange > self.alignmentThreshold:
                                lastCommand = "r"
                                speedVector = [self.autoSpeed, -self.autoSpeed]
                                dataReady = True
                            elif self.angleChange < self.alignmentThreshold:
                                lastCommand = "l"
                                speedVector = [-self.autoSpeed, self.autoSpeed]
                                dataReady = True

                    # wait till its aligned
                    while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  # Add boundaries
                        # keep checking the angle
                        self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                        if self.angleChange < -180:
                            self.angleChange += 360
                        time.sleep(0.01)

                    # stop turning
                    with threadLock:
                        lastCommand = "x"
                        speedVector = [0, 0]
                        dataReady = True
                        # aligned to x axis
                    speechQueue.put("Aligned to the target destination")
                    logging.info("Aligned to the target destination")
                    time.sleep(1)

                    # find distance to Goal
                    distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                    # move to destination
                    speechQueue.put("Moving to the target")
                    logging.info("Moving to Target")

                    with threadLock:
                        lastCommand = "f"
                        speedVector = [self.autoSpeed, self.autoSpeed]
                        dataReady = True

                    initLoc = porterLocation_Global
                    self.distTravelled = 0

                    while (self.distTravelled < self.distQuantise) and autoPilot and not exitFlag.value: #change this to allow for looping the whole block from else till at goal.
                        # self.dX = targetDestination[0] - porterLocation_Global[0]
                        # self.dY = targetDestination[1] - porterLocation_Global[1]
                        # distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                        self.dX = porterLocation_Global[0] - initLoc[0]
                        self.dY = porterLocation_Global[1] - initLoc[1]
                        self.distTravelled = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))

                    with threadLock:
                        lastCommand = "x"
                        speedVector = [0, 0]
                        dataReady = True

                    self.dX = targetDestination[0] - porterLocation_Global[0]
                    self.dY = targetDestination[1] - porterLocation_Global[1]

                speechQueue.put("Successfully arrived at the target")
                logging.info("Successfully arrived at the target")
                autoPilot = False  # turn off autopilot at the end of the maneuver

            else: #if in autopilot mode...
                porterLocation_Global = porterLocation_Local # for now assume local position is the same as global
                    # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER

                # find the required angle change to align to global grid
                if autoPilot and not exitFlag.value:
                    logging.info("Looking for north")
                    speechQueue.put("Looking for north") #vocalise
                    time.sleep(1) #sleep for presentation purposes

                # Orienting to X-axis...
                self.angleChange = 90 - numpy.rad2deg(porterOrientation.value) #find required angle change
                    # right if +ve left if -ve
                if self.angleChange > 180: # if >180 turn left instead of right
                    self.angleChange -= 360

                if autoPilot and not exitFlag.value:
                    with threadLock: #with exclusive global variable access...
                        if self.angleChange > self.alignmentThreshold: #if need to turn right
                            lastCommand = "r" #set the command to be right (for obstacle avoidance purposes)
                            speedVector = [self.autoSpeed, -self.autoSpeed] #set the required speeds
                            dataReady = True #this signals the motor control thread that there is data to be sent
                        elif self.angleChange < self.alignmentThreshold: #if need to turn left
                            lastCommand = "l"
                            speedVector = [-self.autoSpeed, self.autoSpeed]
                            dataReady = True

                # wait till its aligned
                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  #while not aligned
                    # keep calculating the angle change required
                    self.angleChange = 90 - numpy.rad2deg(porterOrientation.value)  # right if +ve left if -ve
                    if self.angleChange > 180:
                        self.angleChange -= 360  # if >180 turn left instead of right
                    time.sleep(0.01) #sleep a bit so that the CPU isnt overloaded

                # stop turning
                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                # aligned to x axis... YAY!
                if autoPilot and not exitFlag.value:
                    speechQueue.put("Aligned to X axis")
                    logging.info("Aligned to X axis")
                    time.sleep(1)

                #find the angle of the goal from north
                self.dX = targetDestination[0] - porterLocation_Global[0]
                self.dY = targetDestination[1] - porterLocation_Global[1]

                if self.dX != 0:
                    self.angleToGoal = numpy.arctan(self.dY / self.dX)
                else:
                    self.angleToGoal = numpy.pi / 2

                self.angleToGoal = numpy.rad2deg(self.angleToGoal)

                # convert angle to 0,2pi
                if self.dX >= 0:  # positive x
                    if self.dY >= 0:
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - self.angleToGoal
                    if self.dY < 0:  # negative y
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - self.angleToGoal

                elif self.dX < 0:  # negative x
                    if self.dY >= 0:  # positive y
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - (180 + self.angleToGoal)
                    if self.dY < 0:  # negative y
                        self.angleToGoal = numpy.rad2deg(porterOrientation.value) - (180 + self.angleToGoal)

                # at this point angleToGoal is defined relative to north
                # if this works remove redundant IF statements. Currently used only for debugging.

                #same as before, finding the angle that needs to be changed
                self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                if self.angleChange < -180:
                    self.angleChange += 360

                if autoPilot and not exitFlag.value:
                    with threadLock:
                        if self.angleChange > self.alignmentThreshold:
                            lastCommand = "r"
                            speedVector = [self.autoSpeed, -self.autoSpeed]
                            dataReady = True
                        elif self.angleChange < self.alignmentThreshold:
                            lastCommand = "l"
                            speedVector = [-self.autoSpeed, self.autoSpeed]
                            dataReady = True

                if autoPilot and not exitFlag.value:
                    speechQueue.put("Looking for the target destination")
                    logging.info("Looking for the target destination")
                    time.sleep(1)

                # wait till its aligned
                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  # Add boundaries
                    # keep checking the angle
                    self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                    if self.angleChange < -180:
                        self.angleChange += 360
                    time.sleep(0.01)

                # stop turning
                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True
                    # aligned to x axis
                if autoPilot and not exitFlag.value:
                    speechQueue.put("Aligned to the target destination")
                    logging.info("Aligned to the target destination")
                    time.sleep(1)

                # find distance to Goal
                distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                # move to destination
                if autoPilot and not exitFlag.value:
                    speechQueue.put("Moving to the target")
                    logging.info("Moving to Target")

                #COMPLETE THIS BIT
                # with threadLock:
                #     lastCommand = "f"
                #     speedVector = [self.autoSpeed, self.autoSpeed]
                #     dataReady = True

                # while (distanceToGoal > 30) and autoPilot and not exitFlag.value: #change this to allow for looping the whole block from else till at goal.
                #     self.dX = targetDestination[0] - porterLocation_Global[0]
                #     self.dY = targetDestination[1] - porterLocation_Global[1]
                #     distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))

                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                if autoPilot and not exitFlag.value:
                    speechQueue.put("Successfully arrived at the target")
                    logging.info("Successfully arrived at the target")
                    autoPilot = False #turn off autopilot at the end of the maneuver

    def turn(self,angleChange):
        global lastCommand
        global speedVector
        global dataReady

        initOrientation = numpy.rad2deg(porterOrientation.value)

        if autoPilot and not exitFlag.value and not obstruction:
            with threadLock:
                if angleChange > self.alignmentThreshold:
                    lastCommand = "r"
                    speedVector = [self.autoSpeed, -self.autoSpeed]
                    dataReady = True
                elif angleChange < self.alignmentThreshold:
                    lastCommand = "l"
                    speedVector = [-self.autoSpeed, self.autoSpeed]
                    dataReady = True

        while (abs(initOrientation - numpy.rad2deg(porterOrientation.value)) < (abs(angleChange) + self.alignmentThreshold)) and autoPilot and not exitFlag.value and not obstruction:  # while not aligned
            time.sleep(0.01)  # sleep a bit so that the CPU isnt overloaded

        # stop turning
        with threadLock:
            lastCommand = "x"
            speedVector = [0, 0]
            dataReady = True


    def moveStraight(self,dist,direction): #direction can be "f" or "b"
        global lastCommand
        global speedVector
        global dataReady

        with threadLock:
            if direction == "f":
                lastCommand = "f"
                speedVector = [self.autoSpeed, self.autoSpeed]
                dataReady = True
            elif direction == "b":
                lastCommand = "b"
                speedVector = [self.autoSpeed, self.autoSpeed]
                dataReady = True

        initLoc = porterLocation_Global
        self.distTravelled = 0

        while (self.distTravelled < dist) and autoPilot and not exitFlag.value and not obstruction:  # change this to allow for looping the whole block from else till at goal.
            self.dX = porterLocation_Global[0] - initLoc[0]
            self.dY = porterLocation_Global[1] - initLoc[1]
            self.distTravelled = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))

        with threadLock:
            lastCommand = "x"
            speedVector = [0, 0]
            dataReady = True

    ###TO BE IMPLEMENTED
    ##AutoPilot motion to be quantised to distQuantise

    def checkquad(self):
        distScore = []  # relative to the current orientation f,b,l,r
        envScore = []
        distScore = self.checkdist(distScore)
        envScore = self.envCheck()
        return self.calcHeuristic(distScore, envScore)

    def checkdist(self, distScore):

        directions = [numpy.rad2deg(porterOrientation.value), 180 - numpy.rad2deg(porterOrientation.value),
                      90 - numpy.rad2deg(porterOrientation.value), 90 + numpy.rad2deg(porterOrientation.value)]

        for direction in directions:
            tempXpos = porterLocation_Global[0] + self.distQuantise * numpy.cos(90 - direction)
            tempYpos = porterLocation_Global[1] + self.distQuantise * numpy.sin(90 - direction)

            tempDX = targetDestination[0] - tempXpos
            tempDY = targetDestination[1] - tempYpos

            tempDist = numpy.sqrt(numpy.square(tempDX) + numpy.square(tempDY))
            distScore.append(tempDist)

        print("dist score is " + str(distScore) + "min dist score is " + str(min(distScore)) + " at " +
              str(distScore.index(min(distScore))))

        return distScore

    def envCheck(self):
        #self.envScore = []

        envScore = [10 * numpy.exp(-2 * USAvgDistances[0] / 10),
                    10 * numpy.exp(-2 * USAvgDistances[5] / 10),
                    10 * numpy.exp(-2 * USAvgDistances[3] / 10),
                    10 * numpy.exp(-2 * USAvgDistances[4] / 10)]
        #Exponential mapping of distance

        print("environment score is " + str(envScore) + " max env score is " + str(max(envScore)) + " at " +
              str(envScore.index(max(envScore))))

        return envScore

    def calcHeuristic(self,distScore, envScore):

        score = []

        for i in range(0, 4):
            if distScore[i] != 0:
                score.append(self.alpha * envScore[i] + self.beta / distScore[i])
            else:
                score.append(self.alpha * envScore[i] + 1)

        #NO MOMENTUM BONUS FOR NOW
        # if len(self.pathmemory) != 0:
        #     print("momentum bonus in the ", self.pathmemory[len(self.pathmemory) - 1], "direction")
        #     score[self.pathmemory[len(self.pathmemory) - 1]] = score[self.pathmemory[
        #         len(self.pathmemory) - 1]] + self.momentumBonus
        # else:
        #     print('start of run, no momentum bonus')

        self.bestScoreIndex = score.index(max(score))

        print("heuristic score is " + str(score))
        print("max heuristic score is " + str(max(score)) + " and its at " + str( self.bestScoreIndex))

        return score

class debugThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.avgRuntime = 0.
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopProfilingInterval = 10
        self.profiling = False

        self.debugServer = False
        self.debugClient = False
        self.dataStore = ""

    def run(self):
        logging.info("Starting %s", self.name)
        while not exitFlag.value:
            self.loopStartFlag()
            if not self.debugServer:
                self.runServer()
            if self.debugServer and not self.debugClient:
                self.waitForClient()
            if self.debugClient:
                try:
                    logging.debug("Sending info to Logger")
                    # Start Flag
                    self.clientConnection.send("#") #0
                    # Time
                    self.clientConnection.send(time.ctime() + ',')
                    # Ultrasonic data
                    self.clientConnection.send(str(USAvgDistances[0]) + ",") #1
                    self.clientConnection.send(str(USAvgDistances[1]) + ",")
                    self.clientConnection.send(str(USAvgDistances[2]) + ",")
                    self.clientConnection.send(str(USAvgDistances[3]) + ",")
                    self.clientConnection.send(str(USAvgDistances[4]) + ",")
                    self.clientConnection.send(str(USAvgDistances[5]) + ",") #6
                    # motor speeds
                    # demanded motor speeds
                    self.clientConnection.send(str(speedVector[0]) + ",") #7
                    self.clientConnection.send(str(speedVector[1]) + ",")
                    #actual speeds
                    self.clientConnection.send(str(wheelSpeeds[0]) + ",") #9
                    self.clientConnection.send(str(wheelSpeeds[1]) + ",")
                    #POSITIONS
                    #target
                    self.clientConnection.send(str(targetDestination[0]) + ",") #11
                    self.clientConnection.send(str(targetDestination[1]) + ",")
                    #Porter_global
                    self.clientConnection.send(str(porterLocation_Global[0]) + ",") #13
                    self.clientConnection.send(str(porterLocation_Global[1]) + ",")
                    #Porter_local
                    self.clientConnection.send(str(porterLocation_Local[0]) + ",") #15
                    self.clientConnection.send(str(porterLocation_Local[1]) + ",")
                    #porterOrientation
                    self.clientConnection.send(str(porterOrientation.value) + ",") #17
                    # AHRS # FOR VALIDATION ONLY
                    # yaw
                    self.clientConnection.send(str(porterOrientation.value) + ",") #18


                    # thread life status
                    self.clientConnection.send(str(threadLock.locked()) + ",") #19
                    # current lock

                    # safety staus
                    self.clientConnection.send(str(safetyOn) + ",") #20
                    # obstruction status
                    self.clientConnection.send(str(obstruction) + ",") #21
                    # data status
                    self.clientConnection.send(str(dataReady)+",") #22

                    self.clientConnection.send(str(orientationWheels.value) + ",")  # 23

                    self.clientConnection.send(str(USAvgDistances[6]) + ",")  # 24
                    self.clientConnection.send(str(USAvgDistances[7]) + ",")
                    self.clientConnection.send(str(USAvgDistances[8]) + ",")
                    self.clientConnection.send(str(USAvgDistances[9]) + ",")
                    self.clientConnection.send(str(USAvgDistances[10]) + ",")
                    self.clientConnection.send(str(USAvgDistances[11]) + ",")
                    self.clientConnection.send(str(USAvgDistances[12]) )  # 30

                    self.clientConnection.send("\n")
                    # self.logOverNetwork()
                    time.sleep(0.2)

                except Exception as e:
                    logging.error("%s", str(e))
                    self.debugClient = False
                    # individual thread states
                    # Program log? for debuging?

    def waitForClient(self):
        # for each connection received create a tunnel to the client
        try:
            logging.debug("Ready for a new client to connect...(5s Timeout)")
            self.clientConnection, self.clientAddress = self.SeverSocket.accept()
            logging.info("Connected by %s", self.clientAddress)
            # send welcome message

            logging.debug("Sending welcome message...")
            self.clientConnection.send('S0')
            logging.debug("Getting data ACK from Client")
            self.dataStore = self.clientConnection.recv(1024)
            if self.dataStore == "S1":
                logging.debug("Client Connection Validated")
                self.debugClient = True
            else:
                logging.warning("Client Connection FAILED :( Try again...")
                self.clientConnection.close()
                self.debugClient = False
        except Exception as e:
            logging.error("%s", str(e))

    def runServer(self):
        logging.info("Setting up sockets...")
        try:
            if (platform == "linux") or (platform == "linux2"):
                HOST = get_ip_address('wlan0')  # socket.gethostbyname(socket.gethostname()) #socket.gethostname()
            else:
                HOST = "localhost"
            PORT = 5003

            # create a socket to establish a server
            logging.debug("Binding the socket...")
            self.SeverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.SeverSocket.bind((HOST, PORT))
            self.SeverSocket.settimeout(5)

            # listen to incoming connections on PORT
            logging.info("Socket opened at %s listening to port %s", HOST, PORT)
            self.SeverSocket.listen(1)
            self.debugServer = True
        except Exception as e:
            logging.error("%s", str(e))
            self.debugServer = False
            logging.debug("Sleeping...")
            time.sleep(5)

    def logOverNetwork(self):
        rootLogger = logging.getLogger('')
        rootLogger.setLevel(logging.DEBUG)
        socketHandler = logging.handlers.SocketHandler('localhost',
                                                       logging.handlers.DEFAULT_TCP_LOGGING_PORT)
        # # don't bother with a formatter, since a socket handler sends the event as
        # # an unformatted pickle
        rootLogger.addHandler(socketHandler)
        #
        # # Now, we can log to the root logger, or any other logger. First the root...
        # logging.info('Jackdaws love my big sphinx of quartz.')
        #
        # # Now, define a couple of other loggers which might represent areas in your
        # # application:
        #
        # logger1 = logging.getLogger('myapp.area1')
        # logger2 = logging.getLogger('myapp.area2')
        #
        # logger1.debug('Quick zephyrs blow, vexing daft Jim.')
        # logger1.info('How quickly daft jumping zebras vex.')
        # logger2.warning('Jail zesty vixen who grabbed pay from quack.')
        # logger2.error('The five boxing wizards jump quickly.')

class motorDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.actionState = 0
        self.profiling = False
        self.oldVector = [0, 0]
        self.hexData = True
        self.lastRandomCode = "R"
        self.inputBuf = ""
        self.pulses = ["",""]
        self.obs = True

    def run(self):
        global speedVector
        global threadLock
        global dataReady
        global USConn1
        global obstruction
        global pulsesQueue

        logging.info("Starting %s", self.name)
        while not exitFlag.value:
            self.loopStartFlag()

            if obstruction:
                if autoPilot and self.obs:
                    if lastSent != [0, 0]:  # ie still moving
                        # pause motion
                        logging.info("Obstacle Detected. Path needs to be recalculated")
                        speechQueue.put("Obstacle Detected. Path needs to be recalculated")
                        self.send_serial_data([0, 0])
                    # else:
                    #     while obstruction:
                    #         time.sleep(0.1)
                    #     speechQueue.put("Obstacle removed. Proceeding to destination")
                    #     logging.info("Obstacle removed. Proceeding to destination")
                    #     self.send_serial_data(speedVector)
                elif autoPilot:
                    logging.info("autopilot command to motor")
                    if lastSent != [0, 0]:  # ie still moving
                        # pause motion
                        logging.info("Obstacle Detected. Waiting for it to go away")
                        speechQueue.put("Obstacle Detected. Waiting for it to go away")
                        self.send_serial_data([0, 0])
                    else:
                        while obstruction:
                            time.sleep(0.1)
                        speechQueue.put("Obstacle removed. Proceeding to destination")
                        logging.info("Obstacle removed. Proceeding to destination")
                        self.send_serial_data(speedVector)
                elif safetyOn: #if not autopilot and the safety is on
                    if lastSent != [0, 0]:
                        logging.debug("Setting Speed Vector")
                        with threadLock:
                            speedVector = [0, 0]
                            dataReady = False
                        self.send_serial_data(speedVector)
                elif lastSent != speedVector: #otherwise...
                    logging.warning("Obstacle Detected But Safety OFF...") #give a warning, but dont do anything to stop
                    self.send_serial_data(speedVector)

            elif dataReady and (lastSent != speedVector): #no obstruction and the new command is different to the last command
                logging.debug("Data Ready")
                try:
                    if not safetyOn:
                        try:
                            logging.info("Trying to send data no safety")
                            self.send_serial_data(speedVector)
                        except Exception as e:
                            logging.error("%s", str(e))
                            if MotorConn.closed:
                                try:
                                    logging.info("Trying to open serial port")
                                    MotorConn.open()
                                except Exception as e:
                                    logging.error("%s", str(e))
                                finally:
                                    logging.info("No Motor Comms... Looping back to listening mode")
                        finally:
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    elif (safetyOn and not USConn1.closed):
                        try:
                            logging.info("Trying to send data")
                            self.send_serial_data(speedVector)
                        except Exception as e:
                            logging.error("%s", str(e))
                            if MotorConn.closed:
                                try:
                                    logging.info("Trying to open serial port")
                                    MotorConn.open()
                                except Exception as e:
                                    logging.error("%s", str(e))
                                finally:
                                    logging.info("No Motor Comms... Looping back to listening mode")
                        finally:
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    else:
                        logging.info("No ultrasonic Comms... HALTING")
                        if speedVector != [0, 0]:
                            logging.debug("Reseting Speed Vector")
                            with threadLock:
                                speedVector = [0, 0]
                                dataReady = False
                        self.send_serial_data(speedVector)

                except Exception as e:
                    logging.error("%s", str(e))

            # READ FROM ARDUINO?
            if MotorConn.inWaiting() > 0:
                #logging.info("data from motor available")
                self.inputBuf = MotorConn.readline()
                self.inputBuf = self.inputBuf.rstrip("\r\n")
                self.pulses = self.inputBuf.split(",")
                pulsesQueue.put(self.pulses)
                #pulsesQueue.get()
                #pulsesQueue.task_done()
                #logging.info("data from motor read")

            if self.profiling:
                time.sleep(0.1)
                # self.read_serial_data()
            else:
                time.sleep(0.001)
                # self.loopEndFlag()
                # self.loopRunTime()


        logging.info("Exiting")

    def send_serial_data(self, sendCommand):
        global lastSent
        logging.info("Sending Speed Vector - %s", str(sendCommand))

        if self.hexData: #construct the command to be sent.
            sendData = "$"
            if sendCommand[0] >= 0:
                sendData += "+"
            else:
                sendData += "-"
            if abs(sendCommand[0]) < 16:
                sendData += "0"
            sendData += hex(abs(sendCommand[0]))[2:]

            if sendCommand[1] >= 0:
                sendData += "+"
            else:
                sendData += "-"
            if abs(sendCommand[1]) < 16:
                sendData += "0"
            sendData += hex(abs(sendCommand[1]))[2:]

            sendData += "R" + "\n"

        else:
            sendData = "+" + str(sendCommand[0]) + "," + str(sendCommand[1]) + "\n"

        try:
            MotorConn.write(sendData)
            logging.info("Successfully sent... - %s", str(sendData))
            lastSent = sendCommand
        except Exception as e:
            logging.error("Sending to Motor failed - %s", str(e))
            # self.actionState = 4

    def read_serial_data(self):
        motorInput = MotorConn.readline()

        logging.debug("motor says %s", motorInput)
        if motorInput == "3\r\n":  # recieved
            self.actionState = 3
            logging.info("Command successfully received by motor...")
        elif motorInput == "2\r\n":
            logging.debug("Motor actuating command...")
            self.actionState = 2
        elif motorInput == "1\r\n":
            logging.debug("Motor actuation finished... ready to accept new data :)")
            self.actionState = 1
        elif motorInput == "5\r\n":
            logging.error("ERROR TRYING TO EXECUTE COMMAND :( ...")
            self.actionState = 5

class usDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata_1 = [0., 0., 0., 0., 0., 0.]
        self.rawUSdata_2 = [0., 0., 0., 0., 0., 0., 0.]

        self.inputBuf = ""
        self.errorCount = 0
        self.profiling = False

    def run(self):
        global speedVector

        logging.info("Starting")
        while not exitFlag.value:
            try:
                self.getUSvector()
                self.mAverage(5)
                self.errorCount = 0
            except Exception as e:
                self.errorCount += 1
                logging.error("%s", str(e))
                if USConn1.closed:
                    try:
                        logging.debug("Trying to open US1 serial port ")
                        USConn1.open()
                    except Exception as e:
                        self.errorCount += 1
                        logging.error("Trying to open Ultrasonic port 1 - %s", str(e))
                        logging.info("No Ultrasonic comms... Looping Back...")

                if UShosts == 2 and USConn2.closed:
                    try:
                        logging.debug("Trying to open US2 serial port")
                        USConn2.open()
                    except Exception as e:
                        self.errorCount += 1
                        logging.error("Trying to open Ultrasonic port 2 - %s", str(e))
                        logging.info("No Ultrasonic comms... Looping Back...")

            if self.errorCount > 3:
                logging.warning("Consecutive Exceptions... pausing execution...")
                self.errorCount = 0
                time.sleep(3)
                logging.info("US resuming")

            self.US_safety_interrupt()

            if self.profiling:
                time.sleep(0.1)
                self.loopEndFlag()
                self.loopRunTime()
            else:
                time.sleep(0.01)

        logging.info("Exiting")

    def getUSvector(self):
        logging.debug("inside getUSVector")
        try:
            logging.debug("Trying to get US data 1")
            self.inputBuf = USConn1.readline()
            self.inputBuf.rstrip("\n\r")
            self.rawUSdata_1 = self.inputBuf.split(",")

            if UShosts == 2:
                logging.debug("Trying to get US data 2")
                self.inputBuf = USConn2.readline()
                self.inputBuf.rstrip("\n\r")
                self.rawUSdata_2 = self.inputBuf.split(",")

        except Exception as e:
            self.errorCount += 1
            logging.error("%s", str(e))

    def mAverage(self, n):
        global USAvgDistances
        global threadLock
        i = 0

        rawUSdata = self.rawUSdata_1 + self.rawUSdata_2

        if len(rawUSdata) == 13:
            with threadLock:
                for i in range(0, len(USAvgDistances)):
                    USAvgDistances[i] += (int(rawUSdata[i]) - USAvgDistances[i]) / n
                if self.profiling:
                    print ("\t" + self.name + " Avg Vector - " + str(USAvgDistances))

    def US_safety_interrupt(self):
        global USAvgDistances
        global lastCommand
        global dataReady
        global obstruction
        global threadLock

        # if safetyOn:
        try:
            if lastCommand == "f":
                if (int(USAvgDistances[0]) < USThresholds[0]):
                    logging.warning("FRONT TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[1]))
                           + ", " + str(int(USAvgDistances[0])) + ", " + str(int(USAvgDistances[2])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastCommand == "b":
                if int(USAvgDistances[5]) < USThresholds[2]:
                    logging.warning("BACK TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[5])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastCommand == "l":
                if int(USAvgDistances[3]) < USThresholds[1]:
                    logging.warning("LEFT SIDE TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[3])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastCommand == "r":
                if int(USAvgDistances[4]) < USThresholds[1]:
                    logging.warning("RIGHT SIDE TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[4])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False
        except Exception as e:
            self.errorCount += 1
            logging.error("%s", str(e))

class ttsThread(MultiThreadBase): #text to speech thread
    def run(self):

        #initialise speech engine
        engine = pyttsx.init()
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate - 50)

        while not exitFlag.value:
            #talk while there are things to be said
            if not speechQueue.empty():
                engine.say(speechQueue.get())
                engine.runAndWait()
            else:
                time.sleep(0.5)

        engine.say("Shutting down")
        engine.runAndWait()

def porterSpeech(speechQueue,exitFlag):
    # initialise speech engine
    engine = pyttsx.init()
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate - 50)

    while not exitFlag.value:
        # talk while there are things to be said
        if not speechQueue.empty():
            engine.say(speechQueue.get())
            engine.runAndWait()
            #speechQueue.task_done()
        else:
            time.sleep(0.5)

        #time.sleep(0.1)

    engine.say("Shutting down")
    engine.runAndWait()

def cameraProcess():  # QR codes and other camera related stuff if doing optical SLAM use a different thread
    pass

class CameraThreadClass(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.pulseData = ["",""] #pulses counted by the motor controller
        self.vanishx = [0, 0, 0, 0, 0]
        self.vanishy = [0, 0, 0, 0, 0]
        self.vanishxVar = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.vanishyVar = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def run(self):
        global cam

        if cam:
            self.vpFromCam()
        else:
            self.vpFromVid()

    def mAverage(self, vpCoord, n):
        i = 0

        if len(vpCoord) == 2:
            for i in range(0, 2):
                vanish[i] += (int(vpCoord[i]) - vanish[i]) / n

        return vanish

    def medianFilter(self, vpCoord, n):
        i = 0
        #global vpValid


        if len(vpCoord) == 2:
            self.vanishx[0] = self.vanishx[1]
            self.vanishx[1] = self.vanishx[2]
            self.vanishx[2] = self.vanishx[3]
            self.vanishx[3] = self.vanishx[4]
            self.vanishx[4] = vpCoord[0]

            self.vanishy[0] = self.vanishy[1]
            self.vanishy[1] = self.vanishy[2]
            self.vanishy[2] = self.vanishy[3]
            self.vanishy[3] = self.vanishy[4]
            self.vanishy[4] = vpCoord[1]

            #print "Unsorted" , self.vanishx, self.vanishy
            sortedx = sorted(self.vanishx)
            sortedy = sorted(self.vanishy)

            #medVar = np.var(self.vanishx), np.var(self.vanishy)

            #print medVar

            #if (medVar[0] > 100) or (medVar[1] > 100):
            #    vpValid = 0
            #else:
            #    vpValid = 1



            #print "Sorted" , sortedx, sortedy
            vanish = (sortedx[2],sortedy[2])

        return vanish

    def varianceFilter(self, vpCoord, n):
        global vpValid
        i = 0

        for i in range(0, n - 1):
            self.vanishxVar[i] = self.vanishxVar[i + 1]
            self.vanishyVar[i] = self.vanishyVar[i + 1]

        self.vanishxVar[n - 1] = vpCoord[0]
        self.vanishyVar[n - 1] = vpCoord[1]

        medVar = np.var(self.vanishxVar), np.var(self.vanishyVar)

        if (medVar[0] > 100) or (medVar[1] > 100):
            vpValid = 0
        else:
            vpValid = 1

        return 0

    def vpFromCam(self):
        global vanish
        capWebcam = cv2.VideoCapture(0)  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam
        if capWebcam.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
            print "error: capWebcam not accessed successfully\n\n"  # if not, print error message
            #logging.error("error: capWebcam not accessed successfully\n\n")
            os.system("pause")

        img2 = np.zeros((540, 960, 3), np.uint8)

        while cv2.waitKey(1) != 27 and capWebcam.isOpened():
            blnFrameReadSuccessfully, img = capWebcam.read()

            try: #try to find vanishing point
                hough_lines = hough_transform(img, False)  #calculate hough lines
                if hough_lines: #if lines found
                    random_sample = getLineSample(hough_lines, 100)  # take a sample of 100 lines
                    intersections = find_intersections(hough_lines, img)  # Find intersections in the sample
                    if intersections:  # if intersections are found
                        grid_size[0] = img.shape[0] // 10 #set the grid size to be 20 by 20
                        grid_size[1] = img.shape[1] // 20
                        #find vanishing points
                        vanishing_point = vp_candidates(img, grid_size, intersections)
                        #returns the best cell

                        vanish2 = self.medianFilter(vanishing_point[0], 4)
                        vanish = self.mAverage(vanish2, 3)
                        # print vanishing_point[0], vanish

                        cv2.circle(img, (vanish[0], 100), 5, (10, 10, 255), thickness=2)
                        cv2.circle(img, (vanish2[0], 100), 5, (210, 255, 10), thickness=2)

                        cv2.circle(img2, (vanish[0], vanish[1]), 1, (10, 10, 255), thickness=2)
                        cv2.circle(img2, (vanish2[0], vanish2[1]), 1, (210, 255, 10), thickness=2)
                        # cv2.drawMarker(img2, (vanish[0], vanish[1]), (10, 10, 255))
                        # cv2.drawMarker(img2, (vanish2[0], vanish2[1]), (210, 255, 10))

                cv2.imshow('vp Image', img)
                cv2.imshow('img2', img2)

            except Exception as e:
                print ("Error - " + str(e))

        cv2.destroyAllWindows()

    def vpFromImg(self):
        filepath = "cor_in.jpg"

        img = cv2.imread(filepath)
        img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)

        #try:
        hough_lines = hough_transform(img, False) #
        if hough_lines:
            random_sample = getLineSample(hough_lines, 100) #take a sample of n lines
            intersections = find_intersections(random_sample, img) #Find intersections in the sample
            if intersections: #if intersections are found
                grid_rows = 2
                grid_columns = 5

                grid_size[0] = img.shape[0] //10
                grid_size[1] = img.shape[1] //20
                vanishing_point = vp_candidates(img, grid_size, intersections)
                print str(vanishing_point)
                #cv2.rectangle(img, (100, 100), (150, 150), (0, 255, 0), 2)
                cv2.circle(img, vanishing_point[0], 5, (10,10,10),thickness=2)
                cv2.imshow('vp Image',img)

        cv2.waitKey(0) != 27
        cv2.destroyAllWindows()

    def vpFromVid(self):
        global vanish

        img2 = np.zeros((540, 960, 3), np.uint8)
        img3 = np.zeros((540, 960, 3), np.uint8)
        capVid = cv2.VideoCapture('cor2small.mp4')  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

        if capVid.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
            #print "error: capVid not accessed successfully\n\n"  # if not, print error message
            logging.error("error: capWebcam not accessed successfully\n\n")
            os.system("pause")

        while cv2.waitKey(1) != 27 and capVid.isOpened():
            blnFrameReadSuccessfully, img = capVid.read()

            try: #try to find vanishing point
                hough_lines = hough_transform(img, False)  #calculate hough lines
                if hough_lines: #if lines found
                    random_sample = getLineSample(hough_lines, 30)  # take a sample of 100 lines
                    intersections = find_intersections(random_sample, img)  # Find intersections in the sample
                    if intersections:  # if intersections are found
                        grid_size[0] = img.shape[0] // 8 #set the grid size to be 20 by 20
                        grid_size[1] = img.shape[1] // 20
                        #find vanishing points
                        vanishing_point = vp_candidates(img, grid_size, intersections)
                        #returns the best cell

                        #vanish2 = self.medianFilter(vanishing_point[0], 4)
                        vanish2 = self.medianFilter(vanishing_point[0], 5)
                        #vanish3 = self.mAverage(vanish2, 5)
                        #print vanishing_point[0], vanish

                        x = self.varianceFilter(vanishing_point[0], 10)

                        if vpValid == 1:
                            cv2.circle(img, (vanish2[0], vanish2[1]), 5, (210, 255, 10), thickness=2)
                            #cv2.circle(img, (vanish2[0], 100), 5, (210, 255, 10), thickness=2)
                        else:
                            cv2.circle(img, (vanish2[0], vanish2[1]), 5, (10, 10, 255), thickness=2)
                            #cv2.circle(img2, (vanish[0], vanish[1]), 1, (10, 10, 255), thickness=2)
                            #cv2.circle(img2, (vanish2[0], vanish2[1]), 1, (210, 255, 10), thickness=2)
                            #cv2.drawMarker(img2, (vanish[0], vanish[1]), (10, 10, 255))
                            #cv2.drawMarker(img2, (vanish2[0], vanish2[1]), (210, 255, 10))

                cv2.imshow('vp Image', img)
                cv2.imshow('img2', img2)


            except Exception as e:
                print ("Error - " + str(e))

        cv2.destroyAllWindows()

###---Function Definitions

def cmdToSpeeds(inputCommand): #convert commands to speed vectors for manual control
    mSpeed = 0
    validateBuffer = (0, 0)
    LMax = 100
    RMax = 100
    LMin = -100
    RMin = -100

    if inputCommand[0] == "x":
        return 0, 0
    elif inputCommand[0] == "m":
        inputCommand.rstrip("\n")
        validateBuffer = str(inputCommand[1:len(inputCommand)]).split(",")
        validateBuffer[0] = int(validateBuffer[0])
        validateBuffer[1] = int(validateBuffer[1])

        if validateBuffer[0] > LMax:
            validateBuffer[0] = LMax
        elif validateBuffer[0] < LMin:
            validateBuffer[0] = LMin

        if validateBuffer[1] > RMax:
            validateBuffer[1] = RMax
        elif validateBuffer[1] < RMin:
            validateBuffer[1] = RMin

        return validateBuffer

    if len(inputCommand) > 1:
        try:
            inputCommand.rstrip("\n")
            mSpeed = int(inputCommand[1:len(inputCommand)])
        except Exception as e:
            logging.error("%s", e)
            mSpeed = 0
    else:
        mSpeed = 20

    if inputCommand[0] == "f":
        return mSpeed, mSpeed  # Left, Right
    elif inputCommand[0] == "b":
        return -mSpeed, -mSpeed
    elif inputCommand[0] == "r":
        return (mSpeed-5), -(mSpeed-5)
    elif inputCommand[0] == "l":
        return -(mSpeed-5), (mSpeed-5)

def get_ip_address(ifname):
    if (platform == "linux") or (platform == "linux2"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        logging.info("Resolving ip address")
        return socket.inet_ntoa(fcntl.ioctl(
            s.fileno(),
            0x8915,  # SIOCGIFADDR
            struct.pack('256s', ifname[:15])
        )[20:24])
    else:
        logging.error("Not linux... cant find ipAddress")

def cmdToDestination(inputCommand):
    #validateBuffer = [0,0]

    if len(inputCommand) > 4:
        try:
            inputCommand.rstrip("\n")
            validateBuffer = str(inputCommand[1:len(inputCommand)]).split(",")
            return int(validateBuffer[0]), int(validateBuffer[0])
        except Exception as e:
            logging.error("%s", e)
    else:
        logging.error("Invalid destination format")


######################################################
######################################################
######Start of the Main Thread!

if __name__ == '__main__':

    multiprocessing.freeze_support()

    mpManager = multiprocessing.Manager()

    wheelSpeeds = mpManager.list([0, 0])
    porterLocation_Global = mpManager.list([0, 0])  # set location to "undefined"
    porterLocation_Local = mpManager.list([0, 0])
    porterLocation_IMU = mpManager.list([0,0])

    qrDic = mpManager.dict()
    vpDict = mpManager.dict()

    logging.info("Starting system...")
    sysRunning = True

    # #Start Speech Process
    # logging.info("Starting speech Process...")
    # speechProcess = multiprocessing.Process(target=porterSpeech,name="Speech Process",args=(speechQueue,exitFlag,))
    # speechProcess.start()
    # processes.append(speechProcess)

    logging.info("Starting speech Thread...")
    speechThread = ttsThread(2, "Speech Thread")
    speechThread.start()
    threads.append(speechThread)

    speechQueue.put("initialising")
    #time.sleep(2)
    #speechQueue.put("trying to run Tracker")

    #Start IMU data Process
    logging.info("Starting Porter Tracker")
    trackerProcess = multiprocessing.Process(name="IMU Process",target=porterTracker, args=(exitFlag,imuEnable, porterLocation_Global, porterOrientation, wheelSpeeds, pulsesQueue,speechQueue,))
    trackerProcess.start()
    processes.append(trackerProcess)
    speechQueue.put("Tracker Running")

    #Start Autopilot thread
    logging.info("Starting Autopilot Thread")
    autoPilotThread = autoPilotThread(3, "Auto Pilot Thread")
    autoPilotThread.start()
    threads.append(autoPilotThread)

    # try to run Debug server if the user wants it
    dataInput = raw_input("Start Debug server... ? (y/n)")
    if dataInput == "y":
        try:
            debugChannel = debugThread(6, "Debug Thread")
            debugChannel.start()
            threads.append(debugChannel)
            logging.info("Running Debug Server")
            # speechQueue.put("Debug Server Running")
        except Exception as e:
            logging.error("%s", str(e))

    # setup serial connection to motor controller
    logging.info("Trying to connect to serial devices")
    dataInput = raw_input("Connect to motor Controller... ? (y/n)")
    if dataInput == "y": #if user wants to connect to the motor controller...
        dataInput = "" #Reset the variable
        logging.info("Trying to connect to motor controller")
        try: #try to connect
            if (platform == "linux") or (platform == "linux2"):
                MotorConn = serial.Serial('/dev/ttyACM0', 19200)
            elif (platform == "win32"):
                MotorConn = serial.Serial('COM7', 19200)

            logging.info('Connected to Motors %s', str(MotorConn))
            serialThread = motorDataThread(4, "Motor thread")
            serialThread.start()
            threads.append(serialThread)
        except Exception as e: #if something goes wrong... tell the user
            logging.error("Unable to establish serial comms to port /dev/ttyACM0")
            logging.error("%s", str(e))

    # Setup serial conn to US controller
    dataInput = raw_input("Connect Ultrasonic Controller... ? (y/n)")
    if dataInput == "y":
        dataInput = ""
        logging.info("Trying to connect to Ultrasonic controller")
        try:
            if (platform == "linux") or (platform == "linux2"):
                USConn1 = serial.Serial('/dev/ttyACM0', 19200)
                logging.info("Connected to Ultrasonic sensors at %s", str(USConn1))
                if UShosts == 2:
                    USConn2 = serial.Serial('/dev/ttyACM1', 19200)
                    logging.info("Connected to Ultrasonic sensors at %s", str(USConn2))

            elif (platform == "win32"):
                USConn1 = serial.Serial('COM3', 19200)

            USthread = usDataThread(5, "Ultrasonic thread")
            USthread.start()
            threads.append(USthread)
            # speechQueue.put("Ultrasonic Sensors Connected")
        except Exception as e:
            print ('Unable to establish serial comms to US device')
            logging.error("%s", str(e))
            # USConnected = False

    # speechQueue.put("Setup Complete")

    # chose input method
    dataInput = raw_input("Local Comms...? (y/n)")
    if dataInput == "n":
        logging.debug("Trying to Setup Remote control")
        localCtrl = False
        # set the server address and port
        logging.info("Setting up sockets...")
        try:
            HOST = get_ip_address('wlan0')  # socket.gethostbyname(socket.gethostname()) #socket.gethostname()
            PORT = 5002

            # create a socket to establish a server
            logging.info("Binding the socket...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind((HOST, PORT))

            # listen to incoming connections on PORT
            logging.info("Socket opened at %s listening to port %s", HOST, PORT)
            s.listen(1)
        except Exception as e:
            # print ("EXCEPTION trying to open Socket - " +str(e))
            logging.error("%s", str(e))
        finally:
            # print ("Local Control")
            logging.debug("Local Control")
            localCtrl = True
    else:
        logging.debug("Local Control mode")

    # Main Control Loop
    while sysRunning: #while the main loop is not in shutdown mode...
        logging.debug("System is running")
        if not localCtrl: #if remote control, wait for client to connect.
            # for each connection received create a tunnel to the client
            logging.info("Ready for a new client to connect...")
            clientConnection, address = s.accept()
            logging.info('Connected by %s', address)
            print 'Connected by', address

            # send welcome message
            print ("Sending welcome message...")
            clientConnection.send('Connection ack')
            dataInput = clientConnection.recv(1024)
            print ("Client says - " + dataInput)
            dataInput = ""

        cmdExpecting = True

        while cmdExpecting:
            print ("Motor commands are expecting...")
            if not localCtrl:
                dataInput = clientConnection.recv(1024) #maybe make this non blocking?
            else:
                dataInput = raw_input("Please Enter Commands on local terminal...\n")
            if dataInput == "e": #shutdown command server
                cmdExpecting = False
                break
            elif dataInput == "q": #initiate system shutdown
                cmdExpecting = False
                sysRunning = False
            else:
                logging.info("Input Command = %s", dataInput)
                if len(dataInput) > 0: #if more than one character read...
                    if (dataInput[0] == "f" or dataInput[0] == "b" or dataInput[0] == "r" \
                                or dataInput[0] == "l" or dataInput[0] == "x" or dataInput[0] == "m") and not autoPilot: #simple commands
                        logging.info("Valid Command")

                        with threadLock:
                            lastCommand = dataInput[0]
                            # commandToTrans()
                            speedVector = cmdToSpeeds(dataInput)
                            dataReady = True

                        if lastCommand == "m":
                            logging.info("MANUAL OVERRIDE!\n")

                    elif dataInput[0] == "s" and not autoPilot: #Toggle safety
                        if safetyOn:
                            dataInput = raw_input("Do you solemnly swear you're up to no good!?..")

                            if len(dataInput) > 0 and dataInput[0] == "Y":
                                print ("May the force be with you... you are going to need it...")
                                print ("(just don't tell Ms Webster about it... >,< )")
                                safetyOn = False
                        else:#if safety is off...turn it on
                            safetyOn = True
                            print ("Mischief managed ;)")

                        dataInput = ""
                        if dataReady != False:
                            with threadLock:
                                dataReady = False

                    elif dataInput[0] == "a": # Engage/Disengage Auto Pilot
                        if autoPilot:
                            logging.info("Turning OFF Autopilot")
                            speechQueue.put("Turning Off Autopilot")
                            autoPilot = False
                            with threadLock:
                                lastCommand = "x"
                                # commandToTrans()
                                speedVector = cmdToSpeeds("x")
                                dataReady = True
                        else:
                            logging.info("Turning ON Autopilot. Send letter 'a' for emergency stop")
                            speechQueue.put("Turning On Autopilot")
                            autoPilot = True
                            safetyOn = True
                            targetDestination = [-100, 100]  # go to global XY = -100,100
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    elif dataInput[0] == "n" :
                        if not autoPilot:
                            logging.info("Turning ON Autopilot. Send letter 'a' for emergency stop")
                            speechQueue.put("Turning On Autopilot")
                            autoPilot = True
                            safetyOn = True
                            logging.info("Setting Destination")
                            targetDestination = cmdToDestination(dataInput)
                            print ("Destination is " + str(targetDestination))
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    else:
                        if dataReady != False:
                            with threadLock:
                                dataReady = False
                        logging.info("Invalid Command")

            dataInput = ""
            print ("")

        if not localCtrl and clientConnection:
            # shut down the server
            clientConnection.close()
            logging.info("Client at %s closed the connection", str(address))

        else: #
            print ("Use q to shutdown system... ")
            print ("Looping back to the start...")


    exitFlag.value = True #instruct all the threads to close

    if not localCtrl:
        logging.info("Shutting down the server at %s...", HOST)
        s.close()
    else:
        logging.info("Shutting Down")

    logging.info("Waiting for threads to close...")
    for t in threads:
        logging.info("Closing %s thread", t)
        t.join()

    for p in processes:
        logging.info("Closing %s process", p)
        p.join()

    logging.info("Exiting main Thread... BYEEEE")

#######END of Program
