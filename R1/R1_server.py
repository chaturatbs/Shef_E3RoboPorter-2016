#
# Module name - R1_server
# Module Description -

# Author     - C. Samarakoon
# Created    - 15/12/2016
# Modified   - 16/12/2016
#

import socket
import serial
import struct
import threading
import Queue
import time
import numpy
import random

from sys import platform
if platform == "linux" or platform == "linux2":
    import fcntl  # linux specific (keep note)
import logging
import logging.handlers

logging.basicConfig(format='%(asctime)s - (%(threadName)s) %(levelname)s: %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p', level=logging.DEBUG)

global lastCommand
global lastOP
global speedVector
global USConnected

global MotorConn
global USConn
global dataReady

global USAvgDistances
global obstruction

global safetyOn

global threadLock
global queueLock
global debugQueue
global autoPilot

autoPilot = False

debug = True
lastCommand = ""
lastOP = ""
lastSent = [0,0]
speedVector = [0,0]
motorConnected = False

dataReady = False

USAvgDistances = [0., 0., 0., 0., 0., 0.]
USThresholds = [100,30,30]
obstruction = False

threadLock = threading.Lock()
queueLock = threading.Lock()
debugQueue = Queue.Queue(10)
threads = []

safetyOn = True


global tMat
global theta
global porterLocation
global porterOrientation

theta = 0.0
tMat = numpy.array([0, 0])
porterLocation = numpy.array([0, 0])
porterOrientation = 0.0  # from starting heading

localCtrl = True
sysRunning = False
cmdExpecting = False
dataInput = ""
exitFlag = 0


def cmdToSpeeds(inputCommand):
    mSpeed = 0

    if inputCommand[0] == "x":
        return 0, 0
    elif inputCommand[0] == "m":
        return str(inputCommand[1:len(inputCommand)]).split(",")

    if len(inputCommand) > 1:
        try:
            inputCommand.rstrip("\n")
            mSpeed = int(inputCommand[1:len(inputCommand)])
        except Exception as e:
            logging.error("%s", e)
        finally:
            mSpeed = 0
    else:
        mSpeed = 20

    if inputCommand[0] == "f":
        return mSpeed, mSpeed #Left, Right
    elif inputCommand[0] == "b":
        return -mSpeed, -mSpeed
    elif inputCommand[0] == "r":
        #mSpeed -= 10
        return mSpeed, -mSpeed
    elif inputCommand[0] == "l":
        #mSpeed -= 10
        return -mSpeed, mSpeed

#Fil in a function based on the dynamics of the robo
def getSafeDist(speed):
    pass

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

#create a class for the data to be sent over ait


class multiThreadBase(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.avgRuntime = 0.
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopDebugInterval = 10
        self.debug = False

    def loopStartFlag(self):
        self.startTime = time.localtime()

    def loopEndFlag(self):
        self.startTime = time.localtime()

    def loopRunTime(self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime) / self.loopDebugInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopDebugInterval:
            #print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            logging.debug("Avg loop Runtime - %s", str(self.avgRuntime))
            self.avgCounter = 0


class autoPilotThread(multiThreadBase):
    def run(self):
        if not autoPilot:
            pass
        else:
            #go 2m forward
            #orient
                #find magnetic north
                #find the required angle change
            #destination = porterLocation + [20,0]
            #convert pulses to distance travelled by each wheel.
            #find the difference
                #difference gives angular change
                    #L-R gives angular change towards right
                    #theta = L-R
            pass

class debugThread(multiThreadBase):

    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.avgRuntime = 0.
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopDebugInterval = 10
        self.debug = False

        self.debugServer = False
        self.debugClient = False
        self.dataStore = ""
        #self.clientConnection = None
        #self.SeverSocket
        #self.clientAddress

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
                    #Start Flag
                    self.clientConnection.send("#")
                    #Time
                    self.clientConnection.send(time.ctime() + ',')
                    #Ultrasonic data
                    self.clientConnection.send(str(USAvgDistances[0]) + ",")
                    self.clientConnection.send(str(USAvgDistances[1]) + ",")
                    self.clientConnection.send(str(USAvgDistances[2]) + ",")
                    self.clientConnection.send(str(USAvgDistances[3]) + ",")
                    self.clientConnection.send(str(USAvgDistances[4]) + ",")
                    self.clientConnection.send(str(USAvgDistances[5]) + ",")
                    # motor speeds
                    # demanded motor speeds
                    self.clientConnection.send(str(speedVector[0]) + ",")
                    self.clientConnection.send(str(speedVector[1]) + ",")
                    # thread life status
                    self.clientConnection.send(str(threadLock.locked()) + ",")
                    # current lock

                    # safety staus
                    self.clientConnection.send(str(safetyOn)+ ",")
                    # obstruction status
                    self.clientConnection.send(str(obstruction)+ ",")
                    # data status
                    self.clientConnection.send(str(dataReady))
                    self.clientConnection.send("\n")
                    #self.logOverNetwork()
                    time.sleep(0.5)
                except Exception as e:
                    logging.error("%s", str(e))
                    self.debugClient = False
                # individual thread states

                # Program log? for debuging?

    def waitForClient(self):
        # for each connection received create a tunnel to the client
        try :
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


class motorDataThread(multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.actionState = 0
        self.debug = False
        self.oldVector = [0, 0]
        self.hexData = True
        self.lastRandomCode = "R"
        # self.dataReady = 0
        # self.startTime = 0.
        # self.endTime = 0.
        # self.avgCounter = 0
        # self.loopDebugInterval = 10
        # self.avgRuntime = 0

    def run(self):
        global speedVector
        global threadLock
        global dataReady
        global USConn

        logging.info("Starting %s", self.name)
        while not exitFlag:
            self.loopStartFlag()

            if obstruction:
                if safetyOn:
                    if lastSent != [0, 0]:
                        logging.debug("Setting Speed Vector")
                        with threadLock:
                            speedVector = [0,0]
                            dataReady = False
                        self.send_serial_data(speedVector)
                elif lastSent != speedVector:
                    logging.warning("Obstacle Detected But Safety OFF...")
                    self.send_serial_data(speedVector)

            elif dataReady and (lastSent != speedVector):
                logging.debug("Data Ready")
                try:
                    if not safetyOn:
                        try:
                            logging.debug("Trying to send data")
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
                                logging.debug("Trying to send data")
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
                    logging.error("%s",str(e))

            #READ FROM ARDUINO?

            if self.debug:
                time.sleep(0.1)
                #self.read_serial_data()
            else:
                time.sleep(0.001)
                # self.loopEndFlag()
                # self.loopRunTime()
        logging.info("Exiting")

    def send_serial_data(self, sendCommand):
        global lastSent
        logging.info("Sending Speed Vector - %s", str(sendCommand))
        #set max min

        if sendCommand[0]>127:
            sendCommand[0] = 127
        if sendCommand[1] > 127 :
            sendCommand[1] = 127

        if sendCommand[0] < -127:
            sendCommand[0] = -127
        if sendCommand[1] < -127:
            sendCommand[1] = -127

        #2s compliment
        # if self.hexData:
        #     sendData = "+"
        #     if sendCommand[0] >= 0:
        #         if sendCommand[0] <10:
        #             sendData += "0" + hex(sendCommand[0])[2:]
        #         else:
        #             sendData += hex(sendCommand[0])[2:]
        #     else:
        #         sendData += hex(((abs(sendCommand[0]) ^ 0xff) + 1) & 0xff)[2:]
        #
        #     #spacer?
        #     sendData+= ""
        #
        #     if sendCommand[1] >= 0:
        #         if sendCommand[1] <10:
        #             sendData += "0" + hex(sendCommand[1])[2:]
        #         else:
        #             sendData += hex(sendCommand[1])[2:]
        #     else:
        #         sendData += hex(((abs(sendCommand[1]) ^ 0xff) + 1) & 0xff)[2:]
        #
        #     sendData += "R" + "\n"
        #
        #     #hex(abs(sendCommand[0]))[2:] + hex(abs(sendCommand[1]))[2:] + "R" + "\n"

        if self.hexData:
            sendData = "$"
            if sendCommand[0] >= 0:
                sendData += "+"
            else:
                sendData += "-"
            if abs(sendCommand[0]) <10:
                sendData += "0"
            sendData += hex(abs(sendCommand[0]))[2:]

            if sendCommand[1] >= 0:
                sendData += "+"
            else:
                sendData += "-"
            if abs(sendCommand[1]) <10:
                sendData += "0"
            sendData += hex(abs(sendCommand[1]))[2:]

            sendData += "R" + "\n"

        else:
            sendData = "+" + str(sendCommand[0]) + "," + str(sendCommand[1]) + "\n"
        MotorConn.write(sendData)

        logging.info("Successfully sent...")
        lastSent = sendCommand
        #self.actionState = 4

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


class usDataThread(multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata = [0., 0., 0., 0., 0., 0.]
        self.inputBuf = ""
        self.errorCount = 0
        self.debug = False

    def run(self):
        global speedVector
        global USConnected

        logging.info("Starting")
        while not exitFlag:
            try:
                self.getUSvector()
                self.mAverage(5)
            except Exception as e:
                self.errorCount += 1
                logging.error("%s",str(e))
                if USConn.closed:
                    try:
                        logging.debug("Trying to open serial port")
                        USConn.open()
                    except Exception as e:
                        self.errorCount += 1
                        logging.error("Trying to open Ultrasonic port - %s", str(e))
                    finally:
                        logging.info("No Ultrasonic comms... Looping Back...")

            if self.errorCount > 3:
                logging.warning("Consecutive Exceptions... pausing execution...")
                self.errorCount = 0
                time.sleep(3)
                logging.info("US resuming")

            self.US_safety_interrupt()
            if self.debug:
                time.sleep(0.1)
                #self.loopEndFlag()
                #self.loopRunTime()
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
            logging.error("%s",str(e))

    def mAverage(self, n):
        global USAvgDistances
        global threadLock
        i = 0
        if len(self.rawUSdata) == 6:
            with threadLock:
                for i in range(0, len(USAvgDistances)):
                    USAvgDistances[i] += (int(self.rawUSdata[i]) - USAvgDistances[i]) / n
                if self.debug:
                    print ("\t" + self.name + " Avg Vector - " + str(USAvgDistances))

    def US_safety_interrupt(self):
        global USAvgDistances
        global lastOP
        global dataReady
        global obstruction
        global threadLock

        #if safetyOn:
        try:
            if lastOP == "f":
                if (int(USAvgDistances[0]) < USThresholds[0]): #or (int(USAvgDistances[1]) < USThresholds[1]) or (int(USAvgDistances[2]) < USThresholds[1]):
                    logging.warning("FRONT TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[0]))
                           + ", " + str(int(USAvgDistances[1])) + ", " + str(int(USAvgDistances[0])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastOP == "b":
                if int(USAvgDistances[5]) < USThresholds[2] :
                    logging.warning("BACK TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[5])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastOP == "l":
                if int(USAvgDistances[3]) < USThresholds[2]:
                    logging.warning("LEFT SIDE TOO CLOSE. STOPPPPP!!!")
                    if obstruction != True:
                        with threadLock:
                            obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[3])))
                elif obstruction != False:
                    with threadLock:
                        obstruction = False

            elif lastOP == "r":
                if int(USAvgDistances[4]) < USThresholds[2]:
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


def simpTransform():
    pass
    # global porterLocation
    # global porterOrientation
    # porterLocation = porterLocation + tMat
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



######Start of the program!
logging.info("Starting system...")
sysRunning = True

# setup serial connection to motor controller
logging.info("Trying to connect to serial devices")

#Setup serial conn to motor controller
dataInput = raw_input("Connect to motor Controller... ? (y/n)")
if dataInput == "y":
    dataInput = ""
    #print("Trying to connect to motor controller")
    logging.info("Trying to connect to motor controller")
    try:
        if (platform == "linux") or (platform == "linux2"):
            MotorConn = serial.Serial('/dev/ttyACM0', 19200)
        elif (platform == "win32"):
            MotorConn = serial.Serial('COM7', 19200)
            #MotorConn.write("*")
        #MotorConn.closed
        #motorConnected = True
        #print ('Connected to serial port /dev/ttyACM0')
        logging.info('Connected to serial port %s', str(MotorConn))
        serialThread = motorDataThread(1, "Motor thread")
        serialThread.start()
        threads.append(serialThread)
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM0')
        #logging.error("Unable to establish serial comms to port /dev/ttyACM0")
        logging.error("%s", str(e))
        #motorConnected = False

#Setup serial conn to US controller
dataInput = raw_input("Connect Ultrasonic Controller... ? (y/n)")
if dataInput == "y":
    dataInput = ""
    #print("Trying to connect to Ultrasonic controller")
    logging.info("Trying to connect to Ultrasonic controller")
    try:
        if (platform == "linux") or (platform == "linux2"):
            USConn = serial.Serial('/dev/ttyACM1', 19200)
        elif (platform == "win32"):
            USConn = serial.Serial('COM3', 19200)
            #MotorConn.write("*")
        logging.info("Connected to serial port /dev/ttyACM1")
        USthread = usDataThread(2, "Ultrasonic thread")
        USthread.start()
        threads.append(USthread)
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM1')
        logging.error("%s", str(e))
        #USConnected = False


dataInput = raw_input("Start Debug server... ? (y/n)")
if dataInput =="y":
    try:
        debugChannel = debugThread(3,"Debug Thread")
        debugChannel.start()
        threads.append(debugChannel)
        logging.info("Running Debug Server")
    except Exception as e:
        logging.error("%s", str(e))


#chose input method
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
        #print ("EXCEPTION trying to open Socket - " +str(e))
        logging.error("%s", str(e))
    finally:
        #print ("Local Control")
        logging.debug("Local Control")
        localCtrl = True
else:
    logging.debug("Local Control mode")

#Main Control Loop
while sysRunning:
    logging.debug("System is running")
    if not localCtrl:
        # for each connection received create a tunnel to the client
        #print ("ready for a new client to connect...")
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

        if dataInput == "e":
            cmdExpecting = False
            break
        elif dataInput == "q":
            cmdExpecting = False
            sysRunning = False
        else:
            logging.info("Input Command = %s", dataInput)
            if len(dataInput) > 0:
                if (dataInput[0] == "f" or dataInput[0] == "b" or dataInput[0] == "r" \
                        or dataInput[0] == "l" or dataInput[0] == "x" or dataInput[0] == "m") and not autoPilot:
                    logging.info("Valid Command")

                    with threadLock:
                        lastOP = dataInput[0]
                        #commandToTrans()
                        speedVector = cmdToSpeeds(dataInput)
                        dataReady = True

                    if lastOP == "m":
                        print ("MANUAL OVERRIDE!\n")

                elif dataInput[0] == "s" and not autoPilot:
                    if safetyOn:
                        dataInput = raw_input("Do you solemnly swear you're up to no good!?..")
                        if dataInput[0] == "Y":
                            print ("May the force be with you... you are going to need it...")
                            print ("(just don't tell Ms Webster about it... >,< )")
                            safetyOn = False
                    else:
                        safetyOn = True
                        print ("Mischief managed ;)")

                    dataInput = ""
                    if dataReady != False:
                        with threadLock:
                            dataReady = False

                elif dataInput[0] == "a" :
                    #Engage/Disengage Auto Pilot
                    if autoPilot:
                        logging.info("Turning OFF Autopilot")
                        a = False
                        with threadLock:
                            lastOP = "x"
                            # commandToTrans()
                            speedVector = cmdToSpeeds("x")
                            dataReady = True
                    else:
                        logging.info("Turning ON Autopilot. Send letter 'a' for emergency stop")
                        autoPilot = True
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

    else:
        print ("Use q to shutdown system... ")
        print ("Looping back to the start...")

exitFlag = 1
if not localCtrl:
    logging.info("Shutting down the server at %s...", HOST)
    s.close()
else:
    logging.info("Shutting Down")

logging.info("Waiting for threads to close...")

for t in threads:
    logging.info("Closing %s thread", t)
    t.join()

logging.info("Exiting main Thread... BYEEEE")
