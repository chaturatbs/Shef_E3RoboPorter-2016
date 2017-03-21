#
# Module name - R4_server
# Module Description -

# Author     - C. Samarakoon
# Created    - 21/03/2017
# Modified   - 21/03/2017
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
import copy
import cv2
import zbar

from msvcrt import getch

#from vpLib import *

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

##--Setup

US_Enable = False
Motor_Enable = False
Cam_Enable = False
Speech_Enable = False
Debug_Enable = False


##--Motor Commands
global lastCommand #holds the last command that was sent to the motor controllers
global speedVector #demanded wheel speeds (left, right)
global dataReady #boolean to let the threads know that data is ready to be sent to the motors
global wheelSpeeds #measured wheel speeds

lastCommand = ""
speedVector = [0, 0]
setSpeedVector = [0,0]
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
UShosts = 1

##--Multi-Threading/Muti-processing
global threadLock #lock to be used when changing global variables
global speechQueue #Queue holding sentences to be spoken
global pulsesQueue #Queue holding the measured wheel encoder pulses
global serverFeedbackQueue
global nodeQueue

threadLock = threading.Lock()
speechQueue = multiprocessing.Queue()
pulsesQueue = multiprocessing.Queue()
serverFeedbackQueue = multiprocessing.Queue()
nodeQueue = multiprocessing.Queue()

threads = [] #Array holding information on the currently running threads
processes = [] #Array holding information on the currently running Processes

# -QR Codes
global QRdetected #Boolean for the QRcode detection status
global QRdata #String read from the QR code

QRdetected = False
QRdata = ""

# -Camera Functions

grid_size = [0,0]
global cam
global vanish
global vpValid

vpValid = multiprocessing.Value('b', False)
vanish = multiprocessing.Value("d",0)


#fps = [0]
#frameNumber = [0]

# -IMU data
#global imu #Handle for the IMU
global imuEnable #Boolean holding whether IMU is connected
imuEnable = multiprocessing.Value('b', False)

##--Auto Pilot
global autoPilot #Boolean for turning on/off autopilot
autoPilot = False #autopilot turned off
global pidEnable
pidEnable = False

global AHRSmode
AHRSmode = "wheel"
# -Porter Localisation

global porterLocation_Global #vector holding global location [x,y] in cm
global porterLocation_Local #vector holding local location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
global targetDestination  #Target location in global coordinates in cm
global distanceToGoal #Distance to goal in cm
global porterLocation_IMU
global roaming
global localised

localised = multiprocessing.Value("b", False)
roaming = multiprocessing.Value("b", False)
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

    global AHRSmode
    #AHRSmode = "wheel"  # can be imu or wheel. Defaults to wheel
    pulseData = ["", ""]  # pulses counted by the motor controller

    # if AHRSmode is not "imu" or "wheel":
    #     AHRSmode = "wheel"

    if platform == "linux" or platform == "linux2":  # if running on linux
        IMUSettingsFile = RTIMU.Settings("mainIMUcal.ini")  # load IMU caliberation file
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
            #print(str(pulseData))
            try:
                pulseData[0] = int((pulseData[0][0]) + str(int("0x" + str(pulseData[0][1:]), 16)))
                pulseData[1] = int((pulseData[1][0]) + str(int("0x" + str(pulseData[1][1:]), 16)))

                porterLocation_Global, wheelSpeeds, orientationWheels.value = movePorter(pulseData[0], pulseData[1], porterLocation_Global,porterOrientation,wheelSpeeds,orientationWheels)
                #print "Location outside is " + str(porterLocation_Global)
                if AHRSmode is "wheel":
                    porterOrientation.value = orientationWheels.value

            except Exception as e:
                logging.error("%s", str(e))


def movePorter(lPulses, rPulses, porterLocation_Global, porterOrientation,wheelSpeeds,orientationWheels):  # function for calculating the distance moved
    # Assume that it doesnt "move" when rotating
    # Need to check if this function is mathematically sound for all r, theta.

    timeInterval = 0.1  # sampling interval of the motor controller
    wheelRadius = 12
    pulsesPerRev = 360
    wheelBaseWidth = 62
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

    #print "Location is " + str(porterLocation_Global)
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
        self.alpha = 20 #
        self.beta = 10000

        #Autopilot Control parameters
        self.looping = True
        self.obs = False
        self.distQuantise = 30  # path re-calculation distance
        self.autoSpeed = 10  # autopilot movement speed
        self.alignmentThreshold = 10  # alignment error in degrees
        self.hScores = []
        self.bestScoreIndex = 0
        self.distThreshold = 20

        self.PID_Tuning = (0.01, 0.01, 0)
        self.vanishsum = 0

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
        global pidEnable
        global nodeQueue

        while not exitFlag.value: #while the system isn't in shutdown mode
            if not autoPilot: #if autopilot is disabled...

                #targetDestination = [100,0]
                #self.hScores = self.checkquad()
                #h_scores = self.hScores

                time.sleep(1) #...sleep for a bit ...zzzzzzz
            elif AHRSmode is "imu" and not imuEnable.value:
                logging.warning("IMU not available. Can't turn on Autopilot... Soz...")
                speechQueue.put("IMU not available. Can't turn on Autopilot...")
                autoPilot = False
            elif self.looping and self.obs:
                #porterLocation_Global = [0,0]  # for now assume local position is the same as global
                # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER
                logging.info("Iteratve Autopilot mode with Obstacle Avoidance")
                speechQueue.put("Iteratve Autopilot mode with Obstacle Avoidance")  # vocalise

                self.dX = targetDestination[0] - porterLocation_Global[0]
                self.dY = targetDestination[1] - porterLocation_Global[1]

                while numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY)) > self.distThreshold and autoPilot and not exitFlag.value:
                # find the angle of the goal from north

                    speechQueue.put("Quantising the environment")
                    logging.info("Quantising The environment")
                    time.sleep(1)

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
                        speechQueue.put("Obstruction, sleeping for a bit")

                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                speechQueue.put("Successfully arrived at the target")
                logging.info("Successfully arrived at the target")
                autoPilot = False  # turn off autopilot at the end of the maneuver

            elif self.looping: #iterative mode

                if not nodeQueue.empty():
                    targetDestination = nodeQueue.get()
                    print ("Destination is " + str(targetDestination))
                    if not localised.value:
                        logging.warning("Not Localised. Motion path unreliable")
                        speechQueue.put("I cant find where I am. I might run into some trouble")

                    #porterLocation_Global = porterLocation_Local  # for now assume local position is the same as global
                    # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER

                    logging.info("Iteratve Autopilot mode")
                    speechQueue.put("Iteratve Autopilot mode enabled")  # vocalise

                    speechQueue.put("Looking for the target destination")
                    logging.info("Looking for the target destination")

                    self.dX = targetDestination[0] - porterLocation_Global[0]
                    self.dY = targetDestination[1] - porterLocation_Global[1]
                    print "target = " + str(targetDestination)
                    print "location = " + str(porterLocation_Global)

                    while numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY)) > self.distThreshold and autoPilot and not exitFlag.value:
                    # find the angle of the goal from north

                        self.dX = targetDestination[0] - porterLocation_Global[0]
                        self.dY = targetDestination[1] - porterLocation_Global[1]

                        print "dx = " + str(self.dX) + " dy = " + str(self.dY) + "\n"
                        if self.dX != 0:
                            if (self.dX > 0 and self.dY >= 0):
                                self.angleToGoal = numpy.pi/2 - numpy.arctan(self.dY / self.dX)
                            elif (self.dX < 0 and self.dY >= 0):
                                self.angleToGoal = -(numpy.pi/2 + numpy.arctan(self.dY / self.dX))
                            elif (self.dX < 0 and self.dY <= 0):
                                self.angleToGoal = -(numpy.pi/2 + numpy.arctan(self.dY / self.dX))
                            elif (self.dX > 0 and self.dY <= 0):
                                self.angleToGoal = numpy.pi/2  - numpy.arctan(self.dY / self.dX)
                        elif self.dY >= 0:
                            self.angleToGoal = 0
                        elif self.dY < 0 :
                            self.angleToGoal = numpy.pi

                        self.angleToGoal = numpy.rad2deg(self.angleToGoal)

                        print "Angle to goal is " + str(self.angleToGoal)
                        # same as before, finding the angle that needs to be changed

                        self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)

                        if self.angleChange < -180:
                            self.angleChange += 360
                        if self.angleChange > 180:
                            self.angleChange -= 360

                        print "Angle change is " + str(self.angleChange)

                        if autoPilot and not exitFlag.value:
                            if (abs(self.angleChange) > self.alignmentThreshold):
                                with threadLock:
                                    if self.angleChange > self.alignmentThreshold:
                                        lastCommand = "r"
                                        speedVector = [self.autoSpeed, -self.autoSpeed]
                                        dataReady = True
                                    elif self.angleChange < self.alignmentThreshold:
                                        lastCommand = "l"
                                        speedVector = [-self.autoSpeed, self.autoSpeed]
                                        dataReady = True
                            else:
                                pidEnable = True
                            # wait till its aligned
                            while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag.value:  # Add boundaries
                                # keep checking the angle
                                self.angleChange = self.angleToGoal - numpy.rad2deg(porterOrientation.value)
                                print "angle change = " + str(self.angleChange)
                                print "Orientation = " +  str(numpy.rad2deg(porterOrientation.value))
                                if self.angleChange < -180:
                                    self.angleChange += 360
                                if self.angleChange > 180:
                                    self.angleChange -= 360
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
                        print "distance to Goal is " + str(distanceToGoal)
                        # move to destination
                        speechQueue.put("Moving to the target")
                        logging.info("Moving to Target")

                        with threadLock:
                            lastCommand = "f"
                            speedVector = [self.autoSpeed, self.autoSpeed]
                            dataReady = True

                        initLoc = copy.deepcopy(porterLocation_Global)
                        self.distTravelled = 0

                        while (self.distTravelled < self.distQuantise) and (distanceToGoal > self.distThreshold) and autoPilot and not exitFlag.value: #change this to allow for looping the whole block from else till at goal.
                            self.dX = targetDestination[0] - porterLocation_Global[0]
                            self.dY = targetDestination[1] - porterLocation_Global[1]
                            distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                            print "Porter Location is " + str(porterLocation_Global)
                            self.dX = porterLocation_Global[0] - initLoc[0]
                            self.dY = porterLocation_Global[1] - initLoc[1]
                            self.distTravelled = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))
                            print "distance Travelled is " + str(self.distTravelled)
                            time.sleep(0.1)

                        self.dX = targetDestination[0] - porterLocation_Global[0]
                        self.dY = targetDestination[1] - porterLocation_Global[1]

                    with threadLock:
                        lastCommand = "x"
                        speedVector = [0, 0]
                        dataReady = True

                    speechQueue.put("Successfully arrived at the target")
                    logging.info("Successfully arrived at the target")

                else:
                    speechQueue.put("No Nodes available")
                    logging.info("No Nodes available")

                autoPilot = False  # turn off autopilot at the end of the maneuver
                pidEnable = False


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

        with threadLock:
            lastCommand = "x"
            speedVector = [0, 0]
            dataReady = True

        autoPilot = False
        pidEnable = False

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
                speedVector = [-self.autoSpeed, -self.autoSpeed]
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

        envScore = [-200/USAvgDistances[0] + 10,
                    -5000 / USAvgDistances[5] + 10,
                    -500 / USAvgDistances[3] + 10,
                    -500 / USAvgDistances[4] + 10]
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

                    #US data 2
                    self.clientConnection.send(str(USAvgDistances[6]) + ",")  # 24
                    self.clientConnection.send(str(USAvgDistances[7]) + ",")
                    self.clientConnection.send(str(USAvgDistances[8]) + ",")
                    self.clientConnection.send(str(USAvgDistances[9]) + ",")
                    self.clientConnection.send(str(USAvgDistances[10]) + ",")
                    self.clientConnection.send(str(USAvgDistances[11]) + ",")
                    self.clientConnection.send(str(USAvgDistances[12]) + ",")  # 30

                    self.clientConnection.send(str(h_scores[0]) + "," )  # 31
                    self.clientConnection.send(str(h_scores[1]) + "," )
                    self.clientConnection.send(str(h_scores[2]) + "," )
                    self.clientConnection.send(str(h_scores[3]) )  # 34

                    self.clientConnection.send("\n")
                    # self.logOverNetwork()
                    time.sleep(0.25)

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
        self.obs = False

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
                        while obstruction and autoPilot and not exitFlag.value:
                            time.sleep(0.1)
                        if not obstruction and autoPilot and not exitFlag.value:
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

        try:
            USConn1.flushInput()
            if UShosts == 2:
                USConn2.flushInput()
        except Exception as e:
            logging.error("%s", str(e))
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
            USConn1.flushInput()
            self.inputBuf = USConn1.readline()
            self.inputBuf.rstrip("\n\r")
            self.rawUSdata_1 = self.inputBuf.split(",")

            if UShosts == 2:
                logging.debug("Trying to get US data 2")
                USConn2.flushInput()
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
        #print rawUSdata

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
            elif lastCommand == "x":
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

# def ():  # QR codes and other camera related stuff if doing optical SLAM use a different thread
#     pass

class PIDThreadClass(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.PID_Tuning = (0.01, 0.01, 0)
        self.vanishsum = 0

    def run(self):
        global setSpeedVector
        global speedVector
        global pidEnable
        global vpValid
        global vanish

        threading.Timer(0.5, self.run).start()

        if pidEnable and vpValid.Value:

            self.vanishsum += vanish.Value
            offset = (self.PID_Tuning[0] * vanish.Value) + (self.PID_Tuning[1] * self.vanishsum)

            if offset >= 0:
                speedVector = [int(setSpeedVector[0] - offset), setSpeedVector[1]]
            else:
                speedVector = [setSpeedVector[0], int(setSpeedVector[1] + offset)]

        else:
            self.vanishsum = 0
            speedVector = setSpeedVector

        with threadLock:
            dataReady = True



        #print datetime.datetime.now(), adjustedSpeed

# class CameraThreadClass(MultiThreadBase):
#     def __init__(self, threadID, name):
#         threading.Thread.__init__(self)
#         self.threadID = threadID
#         self.name = name
#         self.pulseData = ["",""] #pulses counted by the motor controller
#         self.vanishx = [0, 0, 0, 0, 0]
#         self.vanishy = [0, 0, 0, 0, 0]
#         self.vanishxVar = []
#         self.vanishyVar = []
#         self.profiling = True
#
#     def run(self):
#         global cam
#         self.vpFromCAM()

def qrCalibrate(qrProcessor):
    # Instruct user
    print 'To calibrate the RoboPorter set camera to 5 cm away from QR Code'
    print 'Click on the window when ready to continue...'

    # Waiting for user input
    qrProcessor.user_wait()

    # Process one QR Code
    qrProcessor.process_one()

    # Calibrate Camera Focal Length Function
    # When sysmbol detected..
    F = 0

    for symbol in qrProcessor.results:
        # Seperate code into useful data
        data = symbol.data.split(',')
        location = data[0]
        size = int(data[1])

        # Save code corners
        x0 = symbol.location[0][0]
        x1 = symbol.location[1][0]
        x2 = symbol.location[2][0]
        x3 = symbol.location[3][0]
        y0 = symbol.location[0][1]
        y1 = symbol.location[1][1]
        y2 = symbol.location[2][1]
        y3 = symbol.location[3][1]

        # Calculate x and y centre points
        x_centre = (x0 + x1 + x2 + x3) / 4
        y_centre = (y0 + y1 + y2 + y3) / 4

        # Calculate average pixel width
        P_x = (abs(x_centre - x0) + abs(x_centre - x1) + abs(x_centre - x2) + abs(x_centre - x3)) / 2
        P_y = (abs(y_centre - y0) + abs(y_centre - y1) + abs(y_centre - y2) + abs(y_centre - y3)) / 2
        P = (P_x + P_y) / 2

        # Set Code actual size in mm
        W = size

        # Set distance to 1 metre
        D = 50

        # Calculate Focal length of camera
        F = (P * D) / W

        # Print result
        print 'Focal length is', F
        time.sleep(1)

    return F

def qrCodeScanner():
    # create a Processor
    qrProcessor = zbar.Processor()

    # configure the Processor
    qrProcessor.parse_config('enable')

    # initialize the Processor
    device = '/dev/video0'

    # if len(argv) > 1:
    #     device = argv[1]

    qrProcessor.init(device)

    # enable the preview window
    qrProcessor.visible = True

    F = qrCalibrate(qrProcessor)

    while (True):
        print "Ready..."

        # read at least one barcode (or until window closed)
        qrProcessor.process_one()

        # When symbol detected
        for symbol in qrProcessor.results:
            # Seperate code into useful data
            data = symbol.data.split(',')
            location = data[0]
            size = float(data[1])
            # Save code corners
            x0 = symbol.location[0][0]
            x1 = symbol.location[1][0]
            x2 = symbol.location[2][0]
            x3 = symbol.location[3][0]
            y0 = symbol.location[0][1]
            y1 = symbol.location[1][1]
            y2 = symbol.location[2][1]
            y3 = symbol.location[3][1]

            # Calculate x and y centre points
            x_centre = (x0 + x1 + x2 + x3) / 4
            y_centre = (y0 + y1 + y2 + y3) / 4

            # Calculate average pixel width
            P_x = (abs(x_centre - x0) + abs(x_centre - x1) + abs(x_centre - x2) + abs(x_centre - x3)) / 2
            P_y = (abs(y_centre - y0) + abs(y_centre - y1) + abs(y_centre - y2) + abs(y_centre - y3)) / 2
            P = (P_x + P_y) / 2

            # Set Code actual size in mm
            W = size
            D = ((F * W) / P) / 1000
            print 'F = ', F, 'W = ', W, 'P = ', P, 'D = ', ((F * W) / P)
            print 'QR Code scanned:', symbol.data
            print 'RoboPorter is', "%.2fm" % D, 'away from ' '%s' % data[0]
            time.sleep(1)


def vpFromCam():
    global vanish
    global vpValid

    profiling = False
    vanishx = [0,0,0]
    vanishy = [0,0,0]
    varx = [0, 0, 0, 0, 0]
    vary = [0, 0, 0, 0, 0]
    capVid = cv2.VideoCapture(0)

    #capVid = cv2.VideoCapture('testOutsideLab.avi')  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    outvid = cv2.VideoWriter('output.avi', fourcc, 5.0, (640,480))

    # while not capVid.isOpened():
    #     print ("Error Opening Camera")

    if capVid.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
        print "error: capVid not accessed successfully\n\n"  # if not, print error message
        #logging.error("error: capWebcam not accessed successfully\n\n")
        os.system("pause")
    else:
        centre_point = capVid.read()[1].shape[1] / 2

    while capVid.isOpened() and not exitFlag.value :
        blnFrameReadSuccessfully, img = capVid.read()
        #origimg = img
        #print "frame read"
        startTime = time.time()
        sumTime = 0
        #cv2.imshow("input from cam", img)
        #outvid.write(img)

        #print "Image Loaded: " + str(startTime)

        # if len(frameNumber) > 0:
        #     frameNumber.append(frameNumber[len(frameNumber) - 1] + 1)
        # else:
        #     frameNumber[0] = 1

        try: #try to find vanishing point
            hough_lines, startTime, sumTime = vpLib.hough_transform(img, False, startTime, profiling)  #calculate hough lines
            #print str(hough_lines)
            if hough_lines: #if lines found
                random_sample = vpLib.getLineSample(hough_lines, 30)  # take a sample of 100 line
                intersections = vpLib.find_intersections(random_sample, img)  # Find intersections in the sample

                if profiling:
                    duration = time.time() - startTime
                    print "Intersection Time :" + str(duration)
                    sumTime += duration
                    startTime = time.time()

                #print str(intersections)
                if intersections:  # if intersections are found
                    grid_size[0] = img.shape[0] // 8 #set the grid size to be 20 by 20
                    grid_size[1] = img.shape[1] // 20
                    #find vanishing points
                    vanishing_point = vpLib.vp_candidates(img, grid_size, intersections)
                    #returns the best cell
                    #print vanishing_point
                    vanish2, vanishx, vanishy = medianFilter(vanishing_point[0], 3, vanishx, vanishy)
                    varx, vary = varianceFilter(vanishing_point[0], 5, varx, vary)

                    if vpValid.Value == 1:
                        cv2.circle(img, (vanish2[0], vanish2[1]), 5, (210, 255, 10), thickness=2)
                    else:
                        cv2.circle(img, (vanish2[0], vanish2[1]), 5, (10, 10, 255), thickness=2)

                    vanish.Value = int(vanish2[0]-centre_point)
                else:
                    vpValid.Value = 0
            else:
                vpValid.Value = 0

            cv2.imshow('vp Image', img)

            #time.sleep(0.25)

            if profiling:
                duration = time.time() - startTime
                print "Finish Time :" + str(duration)
                sumTime += duration
                startTime = time.time()

            expectedFPS = 1/sumTime

            print "Expected FPS: " + str(expectedFPS)

            #fps.append(expectedFPS)
            # plt.plot(frameNumber, fps)
            # plt.pause(0.05)

            print "----------------------------------------------"

        except Exception as e:
            pass

        #time.sleep(1)

    capVid.release()
    outvid.release()

    cv2.destroyAllWindows()


def medianFilter(vpCoord, n, vanishx, vanishy):
    i = 0
    #global vpValid

    vanish = [0.,0.]
    #print len(vpCoord)
    if len(vpCoord) == 2:
        #print vpCoord
        #print vanishx, vanishy
        vanishx[0] = vanishx[1]
        vanishx[1] = vanishx[2]
        vanishx[2] = vpCoord[0]

        vanishy[0] = vanishy[1]
        vanishy[1] = vanishy[2]
        vanishy[2] = vpCoord[1]

        sortedx = sorted(vanishx)
        sortedy = sorted(vanishy)

        medVanish = (sortedx[1],sortedy[1])


    return medVanish, vanishx, vanishy

def varianceFilter(vpCoord, n, varx, vary):
    global vpValid
    i = 0

    while (len(varx) < n):
        varx.append(0)

    while (len(vary) < n):
        vary.append(0)

    for i in range(0, n - 1):
        varx[i] = varx[i + 1]
        vary[i] = vary[i + 1]

    varx[n - 1] = vpCoord[0]
    vary[n - 1] = vpCoord[1]

    medVar = numpy.var(varx[0:n-1]), numpy.var(vary[0:n-1])

    if (medVar[0] > 1000) or (medVar[1] > 150):
        vpValid.Value = 0
    else:
        vpValid.Value = 1

    return varx, vary


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

def get_ip_address(ifname): #Who is this code based on?
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
    print "input command is " + str(inputCommand)
    if len(inputCommand) > 4:
        try:
            inputCommand.rstrip("\n")
            validateBuffer = str(inputCommand[1:len(inputCommand)]).split(",")
            print "valdate Buffer is " + str(validateBuffer)
            return int(validateBuffer[0]), int(validateBuffer[1])
        except Exception as e:
            logging.error("%s", e)
            return porterLocation_Global[0],porterLocation_Global[1]
    else:
        logging.error("Invalid destination format")
        return porterLocation_Global[0],porterLocation_Global[1]


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
    speechThread = ttsThread(1, "Speech Thread")
    speechThread.start()
    threads.append(speechThread)

    # #logging.info("Starting speech Thread...")
    # camThread = CameraThreadClass(2, "cam Thread")
    # camThread.start()
    # threads.append(camThread )

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
    autoPilotThread = autoPilotThread(2, "Auto Pilot Thread")
    autoPilotThread.start()
    threads.append(autoPilotThread)
    speechQueue.put("Autopilot Waiting")

    # #Start PID thread
    # logging.info("Starting PID Thread")
    # PIDThread = PIDThreadClass(4, "PID Pilot Thread")
    # PIDThread.start()
    # threads.append(PIDThread)

    # try to run Debug server if the user wants it
    #dataInput = raw_input("Start Debug server... ? (y/n)")
    if Debug_Enable: #dataInput == "y":
        try:
            logging.info("Trying to Run Debug Server")
            debugChannel = debugThread(3, "Debug Thread")
            debugChannel.start()
            threads.append(debugChannel)
            logging.info("Running Debug Server")
            # speechQueue.put("Debug Server Running")
        except Exception as e:
            logging.error("%s", str(e))

    # setup serial connection to motor controller
    logging.info("Trying to connect to serial devices")
    #dataInput = raw_input("Connect to motor Controller... ? (y/n)")
    if Motor_Enable: #dataInput == "y": #if user wants to connect to the motor controller...
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
    #dataInput = raw_input("Connect Ultrasonic Controller... ? (y/n)")
    if US_Enable: #dataInput == "y":
        dataInput = ""
        logging.info("Trying to connect to Ultrasonic controller")
        try:
            if (platform == "linux") or (platform == "linux2"):
                USConn1 = serial.Serial('/dev/ttyACM1', 19200)
                logging.info("Connected to Ultrasonic sensors at %s", str(USConn1))
                if UShosts == 2:
                    USConn2 = serial.Serial('/dev/ttyACM2', 19200)
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

    if Cam_Enable:
        logging.info("Starting Cam Process")
        cameraProcess = multiprocessing.Process(name="CAM Process", target=vpFromCam, args=())
        cameraProcess.start()
        processes.append(cameraProcess)
        speechQueue.put("Camera Process Running")

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
                            setSpeedVector = copy.deepcopy(speedVector)
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
                            #targetDestination = [-100, 100]  # go to global XY = -100,100
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False
                    elif dataInput[0] == "n" :
                        nodeQueue.put(cmdToDestination(dataInput))

                        # if not autoPilot:
                        #     logging.info("Turning ON Autopilot. Send letter 'a' for emergency stop")
                        #     speechQueue.put("Turning On Autopilot")
                        #     autoPilot = True
                        #     safetyOn = True
                        #     logging.info("Setting Destination")
                        #     with threadLock:
                        #
                        #         #targetDestination =
                        #     print ("Destination is " + str(targetDestination))
                        #     if dataReady != False:
                        #         with threadLock:
                        #             dataReady = False

                    elif dataInput[0] == "o":
                        if not autoPilot:
                            logging.info("Resetting Orientation and location")
                            orientationWheels.value = 0
                            porterLocation_Global[0] = 0
                            porterLocation_Global[1] = 0
                        else:
                            logging.info("Cant reset localisation. Autopilot engaged.")
                    elif dataInput[0] == "p":
                        print "Porter Location is " + str(porterLocation_Global)
                        print "Porter Orientation is " + str(porterOrientation)
                        print "Target is " + str(targetDestination)

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

    with threadLock:
        lastCommand = "x"
        speedVector = [0, 0]
        dataReady = True

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
