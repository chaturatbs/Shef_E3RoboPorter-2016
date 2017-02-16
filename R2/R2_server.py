#
# Module name - R2_server
# Module Description -

# Author     - C. Samarakoon
# Created    - 26/01/2017
# Modified   - 28/01/2017
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

###---Global Variables-------------


##--Motor Commands
global lastCommand #holds the last command that was sent to the motor controllers
global speedVector #demanded wheel speeds (left, right)
global dataReady #boolean to let the threads know that data is ready to be sent to the motors
global wheelSpeeds #measured wheel speeds

lastCommand = ""
speedVector = [0, 0]
lastSent = [0, 0]
wheelSpeeds = [0, 0]
dataReady = False

# global USConnected
# motorConnected = False

##--Serial Connections
global MotorConn #serial handle for the motor controller
global USConn #serial handle for the Ultrasonic controller

##--Safety
global safetyOn #Boolean for the state of the safety toggle (used only in manual control)
safetyOn = True

##--US data
global USAvgDistances #vector holding the average US distances
global obstruction #Boolean for comminicating whether there is an obstruction in the direction of travel

USAvgDistances = [0., 0., 0., 0., 0., 0.]
obstruction = False
USThresholds = [50, 30, 30] #threasholds for treating objects as obstacles [front,side,back]

##--Multi-Threading/Muti-processing
global threadLock #lock to be used when changing global variables
global queueLock #
global debugQueue #
global speechQueue #Queue holding sentences to be spoken
global pulsesQueue #Queue holding the measured wheel encoder pulses

threadLock = threading.Lock()
queueLock = threading.Lock()
debugQueue = Queue.Queue(10)
speechQueue = multiprocessing.Queue(10)
pulsesQueue = Queue.Queue(120)

threads = [] #Array holding information on the currently running threads
processes = [] #Array holding information on the currently running Processes


# -QR Codes
global QRdetected #Boolean for the QRcode detection status
global QRdata #String read from the QR code

QRdetected = False
QRdata = ""


# -Camera Functions



# -IMU data
global imu #Handle for the IMU
global imuEnable #Boolean holding whether IMU is connected
global globalIMUData #Data Read from the IMU
global globalIMUFusion #AHRS information form the IMU (derived from globalIMUData )

imuEnable = False #IMU is not initialised


if platform == "linux" or platform == "linux2": #if running on linux
    IMUSettingsFile = RTIMU.Settings("RTIMULib") #load IMU caliberation file
    imu = RTIMU.RTIMU(IMUSettingsFile) #initialise the IMU handle
    imuEnable = True #IMU is initialised

##--Auto Pilot
global autoPilot #Boolean for turning on/off autopilot
autoPilot = False #autopilot turned off

# -Porter Localisation
global tMat #
global theta #
global porterLocation_Global #vector holding global location [x,y] in cm
global porterLocation_Local #vector holding local location [x,y] in cm
global porterOrientation #angle from north (between -180,180) in degrees
global targetLocation  #Target location in global coordinates in cm
global distanceToGoal #Distance to goal in cm

tMat = numpy.array([0, 0])
theta = 0.0
targetLocation = [0,0]

porterLocation_Global = numpy.array([-1, -1]) #set location to "undefined"
porterLocation_Local = numpy.array([0, 0])
porterOrientation = 0.0  # from north heading (in degrees?)
distanceToGoal = 0

# -Debug
debug = True #turns on verbose logging to SCREEN

##-System State Variables
localCtrl = True #Local control = commands sent through SSH not TCP/IP
sysRunning = False #
cmdExpecting = False #
exitFlag = False #set exitFlag = True initiate system shutdown
dataInput = "" #Data string from the user (SSH/TCP)

pExitFlag = multiprocessing.Value('b', False)

#function to set exit flags
def setExitFlag(status):
    global exitFlag
    global pExitFlag

    exitFlag = status
    pExitFlag = status

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

def trackPorterData(imu,globalIMUData,globalIMUFusion,pulsesQueue):
    logging.info("Trying to read IMU")
    logging.info("IMU is %s", str(imu))

    if imu.IMUInit():  # if the IMU is initiated...
        logging.info("IMU init successful")  # ...say so on the log
    else:
        logging.info("IMU init failed :/")  # ...or otherwise

    while not exitFlag:  # while the system isn't in shutdown mode
        if imu.IMURead():  # if data is available from the IMU...
            logging.debug("Reading IMU")
            globalIMUData = imu.getIMUData()  # read the IMU data into the global variable
            globalIMUFusion = globalIMUData["fusionPose"]  # extract AHRS fusion info
            time.sleep(imu.IMUGetPollInterval() * 1.0 / 1000.0)  # delay to sync with the recomended poll rate of the IMU

            # if not pulsesQueue.empty(): #if theres data on the pulse Queue...
            #     self.pulseData = pulsesQueue.get() #...get data
            # Convert to distance and do what needs to be done
            #     pulsesQueue.task_done() #set task complete in the Queue. (othewise the system will no be able to shut down)

            # NEED TO ADD SOME STUFF HERE TO TRANSLATE THE MOTION OF THE ROBOT BASED ON THE PULSE DATA
            # if lastCommand == "f" :
            #     pass
            #     #self.moveLocation(int(self.pulseData[0][1:]), int(self.pulseData[1][1:]))


class IMUDataThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.pulseData = ["",""] #pulses counted by the motor controller

    def run(self):
        global globalIMUData
        global globalIMUFusion
        global imu
        global pulsesQueue

        #Log
        logging.info("Trying to read IMU")
        logging.info("IMU is %s", str(imu))

        if imu.IMUInit(): #if the IMU is initiated...
            logging.info("IMU init successful") #...say so on the log
        else:
            logging.info("IMU init failed :/") #...or otherwise

        while not exitFlag: #while the system isn't in shutdown mode
            if imu.IMURead(): #if data is available from the IMU...
                logging.debug("Reading IMU")
                globalIMUData = imu.getIMUData() #read the IMU data into the global variable
                globalIMUFusion = globalIMUData["fusionPose"] #extract AHRS fusion info
                time.sleep(imu.IMUGetPollInterval() * 1.0 / 1000.0) #delay to sync with the recomended poll rate of the IMU

                #porterOrientation = numpy.rad2deg(globalIMUFusion[2])  # get Yaw Data

            #print (str(pulsesQueue.empty()))
            if not pulsesQueue.empty(): #if theres data on the pulse Queue...
                #logging.info("getting data")
                self.pulseData = pulsesQueue.get() #...get data
                #Convert to distance and do what needs to be done
                #print(str(self.pulseData))
                self.moveLocation(int("0x" + str(self.pulseData[0][1:]),16), int("0x" + str(self.pulseData[1][1:]),16))
                pulsesQueue.task_done() #set task complete in the Queue. (othewise the system will no be able to shut down)



    # ONLY INVOKE WHEN MOVING FORWARDS
    def moveLocation(self, lPulses, rPulses): #function for calculating the distance moved
        # Assume that it doesnt "move" when rotating
        # Need to check if this function is mathematically sound for all r, theta.
        global porterLocation_Global
        global wheelSpeeds
        global porterOrientation

        timeInterval = 0.5 #sampling interval of the motor controller
        wheelRadius = 20
        pulsesPerRev = 360

        #find the wheel speeds
        wheelSpeeds[0] = ((lPulses / timeInterval) * 60) / pulsesPerRev
        wheelSpeeds[1] = ((rPulses / timeInterval) * 60) / pulsesPerRev
        #print(str(wheelSpeeds))

        #find the distance moved
        lDist = (2 * numpy.pi * lPulses / pulsesPerRev) * wheelRadius
        rDist = (2 * numpy.pi * rPulses / pulsesPerRev) * wheelRadius

        r = (lDist + rDist) / 2  # take the average of the distances for now.

        # # r is the staight line distance traveled... so find x,y components
        if lastCommand == "f":
            porterLocation_Global[0] = porterLocation_Global[0] + r * numpy.cos(90 - globalIMUFusion[2]) #x component
            porterLocation_Global[1] = porterLocation_Global[1] + r * numpy.sin(90 - globalIMUFusion[2]) #y component

        elif lastCommand == "b":
            porterLocation_Global[0] = porterLocation_Global[0] - r * numpy.cos(90 - globalIMUFusion[2])  # x component
            porterLocation_Global[1] = porterLocation_Global[1] - r * numpy.sin(90 - globalIMUFusion[2])  # y component

        print (porterLocation_Global)
        print (globalIMUFusion[2])

class autoPilotThread(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        # self.globalPosition = [None,None]
        # self.targetPosition = [0,0]
        # self.localPosition = [0,0]
        # self.angleFromNorth = 0
        self.angleToGoal = 0 #angle between the goal and the robot
        self.angleChange = 0 #angle change required
        self.alignmentThreshold = 5 #alignment error in degrees
        # self.distanceToGo = 0
        self.dX = 0
        self.dY = 0
        self.autoSpeed = 10 #autopilot movement speed

        self.distQuantise = 30 #path re-calculation distance

        #path-finding parameters
        self.recPenalty = 0.2 #penalty for recursion
        self.momentumBonus = 0.5 #bonus for momentum conservation
        self.alpha = 1 #
        self.beta = 10

        # tempX = 0
        # tempY = 0

    def run(self):
        global threadLock
        global dataReady
        global speedVector
        global distanceToGoal
        global porterOrientation
        global porterLocation_Global
        global porterLocation_Local
        global lastCommand
        global targetLocation
        global autoPilot
        # global globalIMUFusion

        while not exitFlag: #while the system isn't in shutdown mode
            if not autoPilot: #if autopilot is disabled...
                time.sleep(1) #...sleep for a bit ...zzzzzzz
            else: #if in autopilot mode...
                porterLocation_Global = porterLocation_Local # for now assume local position is the same as global
                    # otherwise put robot into exploration more till it finds a QR code, then position. FOR LATER

                targetLocation = [-100, 100] # go to global XY = -100,100

                # find the required angle change to align to global grid
                logging.info("Looking for north")
                speechQueue.put("Looking for north") #vocalise
                time.sleep(1) #sleep for presentation purposes
                porterOrientation = numpy.rad2deg(globalIMUFusion[2])  # get Yaw Data

                # Orienting to X-axis...
                self.angleChange = 90 - porterOrientation #find required angle change
                    # right if +ve left if -ve
                if self.angleChange > 180: # if >180 turn left instead of right
                    self.angleChange -= 360

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
                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag:  #while not aligned
                        # Add boundaries
                    # keep calculating the angle change required
                    porterOrientation = numpy.rad2deg(globalIMUFusion[2])  # Yaw Data
                    self.angleChange = 90 - porterOrientation  # right if +ve left if -ve
                    if self.angleChange > 180:
                        self.angleChange -= 360  # if >180 turn left instead of right
                    logging.info("r: %f p: %f y: %f", (numpy.rad2deg(globalIMUFusion[0])),
                                     (numpy.rad2deg(globalIMUFusion[1])), (numpy.rad2deg(globalIMUFusion[2]))) #for debugging purposes
                    time.sleep(0.001) #sleep a bit so that the CPU isnt overloaded

                logging.info("r: %f p: %f y: %f", (numpy.rad2deg(globalIMUFusion[0])),
                             (numpy.rad2deg(globalIMUFusion[1])), (numpy.rad2deg(globalIMUFusion[2])))

                # stop turning
                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                # aligned to x axis... YAY!
                speechQueue.put("Aligned to X axis")
                logging.info("Aligned to X axis")
                time.sleep(1)

                #find the angle of the goal from north
                self.dX = targetLocation[0] - porterLocation_Global[0]
                self.dY = targetLocation[1] - porterLocation_Global[1]

                if self.dX != 0:
                    self.angleToGoal = numpy.arctan(self.dY / self.dX)
                else:
                    self.angleToGoal = numpy.pi / 2

                self.angleToGoal = numpy.rad2deg(self.angleToGoal)

                porterOrientation = numpy.rad2deg(globalIMUFusion[2])

                # convert angle to 0,2pi
                if self.dX >= 0:  # positive x
                    if self.dY >= 0:
                        self.angleToGoal = porterOrientation - self.angleToGoal
                    if self.dY < 0:  # negative y
                        self.angleToGoal = porterOrientation - self.angleToGoal

                elif self.dX < 0:  # negative x
                    if self.dY >= 0:  # positive y
                        self.angleToGoal = porterOrientation - (180 + self.angleToGoal)
                    if self.dY < 0:  # negative y
                        self.angleToGoal = porterOrientation - (180 + self.angleToGoal)

                # at this point angleToGoal is defined relative to north
                # if this works remove redundant IF statements. Currently used only for debugging.

                #same as before, finding the angle that needs to be changed
                self.angleChange = self.angleToGoal - porterOrientation
                if self.angleChange < -180:
                    self.angleChange += 360

                with threadLock:
                    if self.angleChange > self.alignmentThreshold:
                        lastCommand = "r"
                        speedVector = [self.autoSpeed, -self.autoSpeed]
                        dataReady = True
                    elif self.angleChange < self.alignmentThreshold:
                        lastCommand = "l"
                        speedVector = [-self.autoSpeed, self.autoSpeed]
                        dataReady = True

                speechQueue.put("Looking for the target destination")
                logging.info("Looking for the target destination")
                time.sleep(1)

                # wait till its aligned
                while (abs(self.angleChange) > self.alignmentThreshold) and autoPilot and not exitFlag:  # Add boundaries
                    # keep checking the angle
                    logging.info("r: %f p: %f y: %f", (numpy.rad2deg(globalIMUFusion[0])),
                                 (numpy.rad2deg(globalIMUFusion[1])), (numpy.rad2deg(globalIMUFusion[2])))
                    porterOrientation = numpy.rad2deg(globalIMUFusion[2])  # Yaw Data
                    self.angleChange = self.angleToGoal - porterOrientation
                    if self.angleChange < -180:
                        self.angleChange += 360
                    time.sleep(0.001)

                logging.info("r: %f p: %f y: %f", (numpy.rad2deg(globalIMUFusion[0])),
                             (numpy.rad2deg(globalIMUFusion[1])), (numpy.rad2deg(globalIMUFusion[2])))
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

                #COMPLETE THIS BIT
                # with threadLock:
                #     lastCommand = "f"
                #     speedVector = [self.autoSpeed, self.autoSpeed]
                #     dataReady = True

                # while (distanceToGoal > 30) and autoPilot and not exitFlag: #change this to allow for looping the whole block from else till at goal.
                #     self.dX = targetLocation[0] - porterLocation_Global[0]
                #     self.dY = targetLocation[1] - porterLocation_Global[1]
                #     distanceToGoal = numpy.sqrt(numpy.square(self.dX) + numpy.square(self.dY))

                with threadLock:
                    lastCommand = "x"
                    speedVector = [0, 0]
                    dataReady = True

                speechQueue.put("Successfully arrived at the target")
                logging.info("Successfully arrived at the target")
                autoPilot = False #turn off autopilot at the end of the maneuver

    ###TO BE IMPLEMENTED
    ##AutoPilot motion to be quantised to distQuantise

    def checkquad(self):
        distScore = []  # relative to the current orientation f,b,l,r
        envScore = []
        distScore = self.checkdist(distScore)
        envScore = self.envCheck()
        return self.calcHeuristic(distScore, envScore)

    def checkdist(self, distScore):

        directions = [porterOrientation, 180 - porterOrientation,
                      90 - porterOrientation, 90 + porterOrientation]

        for direction in directions:
            tempXpos = porterLocation_Global[0] + self.distQuantise * numpy.cos(90 - direction)
            tempYpos = porterLocation_Global[1] + self.distQuantise * numpy.sin(90 - direction)

            tempDX = targetLocation[0] - tempXpos
            tempDY = targetLocation[1] - tempYpos

            tempDist = numpy.sqrt(numpy.square(tempDX) + numpy.square(tempDY))
            distScore.append(tempDist)

        print("dist score is " + str(distScore))
        print("min dist score is " + str(min(distScore)) + " and its at " +
              str(distScore.index(min(distScore))))

        return distScore.index(min(distScore))

    def envCheck(self):
        #self.envScore = []

        envScore = [min(USAvgDistances[:2]),USAvgDistances[3],USAvgDistances[4],USAvgDistances[5]]
        #map this non-linearly

        print("environment score is " + str(envScore))
        print("max env score is " + str(max(envScore)) + " and its at " +
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

        print("heuristic score is " + str(score))
        print("max heuristic score is " + str(max(score)) + " and its at " +
              str(score.index(max(score))))
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
        # self.clientConnection = None
        # self.SeverSocket
        # self.clientAddress

    def run(self):
        logging.info("Starting %s", self.name)
        while not exitFlag:
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
                    self.clientConnection.send(str(targetLocation[0]) + ",") #11
                    self.clientConnection.send(str(targetLocation[1]) + ",")
                    #Porter_global
                    self.clientConnection.send(str(porterLocation_Global[0]) + ",") #13
                    self.clientConnection.send(str(porterLocation_Global[1]) + ",")
                    #Porter_local
                    self.clientConnection.send(str(porterLocation_Local[0]) + ",") #15
                    self.clientConnection.send(str(porterLocation_Local[1]) + ",")
                    #porterOrientation
                    self.clientConnection.send(str(porterOrientation) + ",") #17
                    # AHRS # FOR VALIDATION ONLY
                    # yaw
                    self.clientConnection.send(str(globalIMUFusion[2]) + ",") #18


                    # thread life status
                    self.clientConnection.send(str(threadLock.locked()) + ",") #19
                    # current lock

                    # safety staus
                    self.clientConnection.send(str(safetyOn) + ",") #20
                    # obstruction status
                    self.clientConnection.send(str(obstruction) + ",") #21
                    # data status
                    self.clientConnection.send(str(dataReady)) #22

                    self.clientConnection.send("\n")

                    # self.logOverNetwork()
                    time.sleep(0.1)
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
        # self.dataReady = 0
        # self.startTime = 0.
        # self.endTime = 0.
        # self.avgCounter = 0
        # self.loopProfilingInterval = 10
        # self.avgRuntime = 0

    def run(self):
        global speedVector
        global threadLock
        global dataReady
        global USConn
        global obstruction
        global pulsesQueue

        logging.info("Starting %s", self.name)
        while not exitFlag:
            self.loopStartFlag()

            if obstruction:
                if autoPilot:
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
                    elif (safetyOn and not USConn.closed):
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
        self.rawUSdata = [0., 0., 0., 0., 0., 0.]
        self.inputBuf = ""
        self.errorCount = 0
        self.profiling = False

    def run(self):
        global speedVector
        # global USConnected

        logging.info("Starting")
        while not exitFlag:
            try:
                self.getUSvector()
                self.mAverage(5)
                self.errorCount = 0
            except Exception as e:
                self.errorCount += 1
                logging.error("%s", str(e))
                if USConn.closed:
                    try:
                        logging.debug("Trying to open serial port")
                        USConn.open()
                    except Exception as e:
                        self.errorCount += 1
                        logging.error("Trying to open Ultrasonic port - %s", str(e))
                        logging.info("No Ultrasonic comms... Looping Back...")

            if self.errorCount > 3:
                logging.warning("Consecutive Exceptions... pausing execution...")
                self.errorCount = 0
                time.sleep(3)
                logging.info("US resuming")

            self.US_safety_interrupt()

            if self.profiling:
                time.sleep(0.1)
                # self.loopEndFlag()
                # self.loopRunTime()
            else:
                time.sleep(0.01)

        logging.info("Exiting")

    def getUSvector(self):
        logging.debug("inside getUSVector")
        try:
            logging.debug("Trying to get US data")
            self.inputBuf = USConn.readline()
            self.inputBuf.rstrip("\n\r")
            self.rawUSdata = self.inputBuf.split(",")
        except Exception as e:
            self.errorCount += 1
            logging.error("%s", str(e))

    def mAverage(self, n):
        global USAvgDistances
        global threadLock
        i = 0
        if len(self.rawUSdata) == 6:
            with threadLock:
                for i in range(0, len(USAvgDistances)):
                    USAvgDistances[i] += (int(self.rawUSdata[i]) - USAvgDistances[i]) / n
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


def porterSpeech(speechQueue,pExitFlag):
    # initialise speech engine
    engine = pyttsx.init()
    rate = engine.getProperty('rate')
    engine.setProperty('rate', rate - 50)

    while not pExitFlag.value:
        # talk while there are things to be said
        if not speechQueue.empty():
            engine.say(speechQueue.get())
            engine.runAndWait()
            #speechQueue.task_done()
        else:
            time.sleep(0.5)

    engine.say("Shutting down")
    engine.runAndWait()

#
# class ttsProcess(multiprocessing.Process): #text to speech thread
#     def __init__(self,name):
#         self.name = name
#
#     def run(self):
#         self.target = self.talk()
#
#     def talk(self):
#         global speech
#         # initialise speech engine
#         engine = pyttsx.init()
#         rate = engine.getProperty('rate')
#         engine.setProperty('rate', rate - 50)
#
#         while not exitFlag:
#             # talk while there are things to be said
#             if not speechQueue.empty():
#                 engine.say(speechQueue.get())
#                 engine.runAndWait()
#                 #speechQueue.task_done()
#             else:
#                 time.sleep(0.5)
#

def cameraProcess():  # QR codes and other camera related stuff if doing optical SLAM use a different thread
    pass


###---Function Definitions

def IMU_Init(): #initialise IMU
    global imu
    global imuEnable

    logging.info("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        logging.error("IMU Init Failed")
        # speechQueue.put("IMU Initiation Failed")
        imuEnable = False
    else: #if connection successful...
        logging.info("IMU Init Succeeded")
        # speechQueue.put("IMU Successfully Initiated")
        imu.setSlerpPower(0.02)
        imu.setGyroEnable(True)
        imu.setAccelEnable(True)
        imu.setCompassEnable(True)
        imuEnable = True
        poll_interval = imu.IMUGetPollInterval()
        logging.debug("Recommended Poll Interval: %dmS\n" % poll_interval)


def simpTransform():
    pass
    # global porterLocation_Global
    # global porterOrientation
    # porterLocation_Global = porterLocation_Global + tMat
    # porterOrientation = porterOrientation + theta


def commandToTrans():
    pass
    # if lastCommand[0] == "F":
    #     tMat[0] = lastCommand[1:len(lastCommand)]
    # elif lastCommand[0] == "B":
    #     tMat[1] = lastCommand[1:len(lastCommand)]
    # elif lastCommand[0] == "L":
    #     theta = lastCommand[1:len(lastCommand)]
    # elif lastCommand[0] == "R":
    #     theta = (-1) * lastCommand[1:len(lastCommand)]


# Fil in a function based on the dynamics of the robot
def getSafeDist(speed):
    pass


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


######################################################
######################################################
######Start of the Main Thread!

if __name__ == '__main__':

    logging.info("Starting system...")
    sysRunning = True

    IMU_Init() #initialise IMU

    #Start IMU data thread
    logging.info("Starting IMU Thread")
    imuThread = IMUDataThread(1, "IMU Thread")
    imuThread.start()
    threads.append(imuThread) #add thread to the thread queue. This MUST be done for every queue

    #Start Speech thread
    logging.info("Starting speech Process...")

    speechProcess = multiprocessing.Process(target=porterSpeech,name="Speech Process",args=(speechQueue,pExitFlag,))#ttsProcess("Speech Process")
    speechProcess.start()
    processes.append(speechProcess)


    #speechProcess.run()
    #threads.append(speechProcess)

    speechQueue.put("initialising")

    pDataManager = multiprocessing.Manager()
    qrDic = pDataManager.dict()
    vpDict = pDataManager.dict()


    #Start Autopilot thread
    logging.info("Starting Autopilot Thread")
    autoPilotThread = autoPilotThread(3, "Auto Pilot Thread")
    autoPilotThread.start()
    threads.append(autoPilotThread)

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
                USConn = serial.Serial('/dev/ttyACM1', 19200)
            elif (platform == "win32"):
                USConn = serial.Serial('COM3', 19200)

            logging.info("Connected to Ultrasonic sensors at %s", str(USConn))
            USthread = usDataThread(5, "Ultrasonic thread")
            USthread.start()
            threads.append(USthread)
            # speechQueue.put("Ultrasonic Sensors Connected")
        except Exception as e:
            print ('Unable to establish serial comms to port /dev/ttyACM1')
            logging.error("%s", str(e))
            # USConnected = False

    #try to run Debug server if the user wants it
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
                dataInput = clientConnection.recv(1024)
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
                            if dataInput[0] == "Y":
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
                            if dataReady != False:
                                with threadLock:
                                    dataReady = False

                    elif dataInput[0] == "n" and autoPilot:
                        # put the code for setting destination nodes here
                        pass

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


    exitFlag = True #instruct all the threads to close
    pExitFlag.value = True

    if not localCtrl:
        logging.info("Shutting down the server at %s...", HOST)
        s.close()
    else:
        logging.info("Shutting Down")

    logging.info("Waiting for threads to close...")
    for t in threads:
        logging.info("Closing %s thread", t)
        t.join()

    #speechProcess.join()
    for p in processes:
        logging.info("Closing %s process", p)
        p.join()

    logging.info("Exiting main Thread... BYEEEE")

#######END of Program
