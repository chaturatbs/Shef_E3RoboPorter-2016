#
# Module name - RC_server
# Module Description -
#       A socket server using python socket modules.
#       Used to receive data from a client over a network connection (wifi/ethernet)
#       This module will take receive data from the client terminal and send appropriate commands to the motor Arduino
#
# Author     - C. Samarakoon
# Created    - 18/10/2016
# Modified   - 26/11/2016
#

import socket
import serial
import fcntl  # linux specific (keep note)
import struct
import threading
import time
import numpy

global lastCommand
lastCommand = ""
global lastOP
lastOP = ""
global speedVector
speedVector = [0,0]

global motorConnected
motorConnected = False
global USConnected
USConnected = False

global MotorConn
global USConn

global dataReady
dataReady = False
global USAvgDistances
USAvgDistances = [0., 0., 0., 0., 0., 0.]
USThresholds = [100,70,50]
global obstruction
obstruction = False
global threadSync
threadSync = True
global safetyOn
safetyOn = True

dataInput = ""
exitFlag = 0

global tMat
global theta
global porterLocation
global porterOrientation

theta = 0.0
tMat = numpy.array([0, 0])
porterLocation = numpy.array([0, 0, 0])
porterOrientation = 0.0  # from starting heading
localCtrl = True
sysRunning = False
cmdExpecting = False


def cmdToSpeeds(inputCommand):
    global mSpeed

    if inputCommand[0] == "x":
        return 0, 0
    elif inputCommand[0] == "m":
        return str(inputCommand[1:len(inputCommand)]).split(",")

    if len(inputCommand) > 1:
        try:
            inputCommand.rstrip("\n")
            mSpeed = int(inputCommand[1:len(inputCommand)])
        except Exception as e:
            print ("Error parsing input - " + str(e))
        finally:
            mSpeed = 0
    else:
        mSpeed = 20

    if inputCommand[0] == "f":
        return mSpeed, mSpeed #Left, Right
    elif inputCommand[0] == "b":
        return -mSpeed, -mSpeed
    elif inputCommand[0] == "r":
        return mSpeed, -mSpeed
    elif inputCommand[0] == "l":
        return -mSpeed, mSpeed

#Fil in a function based on the dynamics of the robo
def getSafeDist(speed):
    pass

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Resolving ip address")
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])


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
        pass

    def loopEndFlag(self):
        self.startTime = time.localtime()
        pass

    def loopRunTime(self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime) / self.loopDebugInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopDebugInterval:
            print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            self.avgCounter = 0


class motorDataThread(multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.actionState = 0
        self.debug = False
        self.oldVector = [0, 0]
        # self.dataReady = 0
        # self.startTime = 0.
        # self.endTime = 0.
        # self.avgCounter = 0
        # self.loopDebugInterval = 10
        # self.avgRuntime = 0

    def run(self):
        global speedVector
        global safetyOn

        print "Starting " + self.name
        while not exitFlag:
            self.loopStartFlag()

            if obstruction and speedVector != [0,0]:
                print ("\t" + self.name + " OBSTRUCTION! :( ")
                if safetyOn:
                    #self.torqueStop(speedVector, 0.3)
                    speedVector = [0,0]
                    self.send_serial_data(speedVector)
                else:
                    print ("\t \t" + self.name
                           + " OH MY GOD IM GOING TO HIT SOMETHING! >.< (I hope its a person...mmmm)")
            elif not USConnected and safetyOn:
                print ("\t" + self.name + " No ultrasonic Comms... Halting")
                if safetyOn:
                    #self.torqueStop(speedVector, 0.3)
                    speedVector = [0, 0]
                    self.send_serial_data(speedVector)
            # elif lastOP == "x":
            #     #self.torqueStop(speedVector, 0.3)d
            #     speedVector = [0, 0]
                self.send_serial_data(speedVector)
            # elif not threadSync:
            #     pass
                #print ("\t" + self.name + " Threads not Synchronising... not safe to move! :( ")
                #speedVector = [0, 0]
                #self.send_serial_data(speedVector)
            elif dataReady:
                print ("\t" + self.name + " Data Ready")
                try:
                    print ("\t" + self.name + " Trying to send data")
                    self.send_serial_data(speedVector)
                except Exception as e:
                    print ("\t" + self.name + ": ERROR - " + str(e))
                    try:
                        print ("\t" + self.name + ": Trying to open serial port")
                        MotorConn.open()
                        # serialConnected = True
                    except Exception as e:
                        print("\t" + self.name + ": ERROR trying to open motor port :( : " + str(e))
                    finally:
                        print ("\t" + self.name + ": No Motor Comms... Looping back to listening mode")

            if self.debug:
                time.sleep(0.1)
                self.read_serial_data()
            else:
                time.sleep(0.001)
                # self.loopEndFlag()
                # self.loopRunTime()
        print "Exiting " + self.name

    def send_serial_data(self, sendCommand):
        global dataReady

        #if sendCommand != self.oldVector:
        print("\t" + self.name + ": Sending Speed Vector - " + str(sendCommand))
        #MotorConn.write(int(255))
        MotorConn.write(str(sendCommand[0]))
        MotorConn.write(",")
        MotorConn.write(str(sendCommand[1]))
        MotorConn.write("\n")
        #MotorConn.write()
        dataReady = False
        if self.debug:
            print("\t" + self.name + ": Successfully sent...")
        self.actionState = 4
        #self.oldVector = sendCommand
        #else:
        #    pass

    def read_serial_data(self):
        motorInput = MotorConn.readline()
        if self.debug:
            print ("\t" + self.name + ": motor says " + motorInput)
        if motorInput == "3\r\n":  # recieved
            self.actionState = 3
            print ("\t" + self.name + " - Command successfully received by motor...")
        elif motorInput == "2\r\n":
            if self.debug:
                print ("\t" + self.name + "- Motor actuating command...")
            self.actionState = 2
        elif motorInput == "1\r\n":
            if self.debug:
                print ("\t" + self.name + "- Motor actuation finished... ready to accept new data :) ")
            self.actionState = 1
        elif motorInput == "5\r\n":
            print ("\t" + self.name + "- ERROR TRYING TO EXECUTE COMMAND :( ...")
            self.actionState = 5

            #
            # def loopStartFlag (self):
            #     self.startTime = time.localtime()
            #     pass
            #
            # def loopEndFlag (self):
            #     self.startTime = time.localtime()
            #     pass
            #
            # def loopRunTime (self):
            #     self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime)/self.loopDebugInterval
            #     self.avgCounter += 1
            #
            #     if self.avgCounter == self.loopDebugInterval :
            #         print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            #         self.avgCounter = 0

    def torqueStop(self, speedVector, sleepTime):
        speedNeutral = [0,0]
        speedNeutral[0] = (-1)*speedVector[0]
        speedNeutral[1] = (-1) * speedVector[1]
        print ("tstop - " + str(speedNeutral))
        self.send_serial_data(speedNeutral)
        print ("\t" + self.name + "Torque Stop!")
        time.sleep(sleepTime)


class usDataThread(multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata = [0., 0., 0., 0., 0., 0.]
        self.inputBuf = ""
        self.errorCount = 0
        self.debug = False
        #self.USThresholds = [100,70,50]

    def run(self):
        global speedVector
        global threadSync

        print "Starting " + self.name
        while not exitFlag:
            if self.debug:
                self.loopStartFlag()

            try:
                self.getUSvector()
                self.mAverage(5)
            except Exception as e:
                self.errorCount += 1
                print ("\t" + self.name + ": ERROR - " + str(e))
                try:
                    if self.debug:
                        print ("\t" + self.name + ":Trying to open serial port")
                    USConn.open()
                    #serialConnected = True
                    threadSync = True
                except Exception as e:
                    self.errorCount += 1
                    print("\t" + self.name + ": ERROR trying to open Ultrasonic port :( : " + str(e))
                finally:
                    print ("\t" + self.name + ": No Ultrasonic comms... Looping Back... ")
                    USConnected = False

            if self.errorCount > 3:
                print ("Consecutive Exceptions... pausing execution...")
                self.errorCount = 0
                threadSync = False
                time.sleep(3)
                print ("US resuming")
                threadSync = True

            self.US_safety_interrupt()
            if self.debug:
                time.sleep(0.1)
                self.loopEndFlag()
                self.loopRunTime()
            else:
                time.sleep(0.01)

        print "Exiting " + self.name

    def getUSvector(self):
        if self.debug:
            print ("\t" + self.name + " inside getUSVector")
        try:
            if self.debug:
                print ("\t" + self.name + ": trying to get US data")
            self.inputBuf = USConn.readline()
            self.inputBuf.rstrip("\n\r")
            self.rawUSdata = self.inputBuf.split(",")
            if self.debug:
                print ("\t" + self.name + " US Raw Vector - " + str(self.rawUSdata))
        except Exception as e:
            self.errorCount += 1
            print ("\t" + self.name + ": ERROR trying to get US data - " + str(e))

    def mAverage(self, n):
        global USAvgDistances
        i = 0
        if len(self.rawUSdata) == 6:
            for i in range(0, len(USAvgDistances)):
                USAvgDistances[i] += (int(self.rawUSdata[i]) - USAvgDistances[i]) / n
            if self.debug:
                print ("\t" + self.name + " Avg Vector - " + str(USAvgDistances))

    def US_safety_interrupt(self):
        global USAvgDistances
        global lastOP
        global dataReady
        global obstruction
        global threadSync

        threadSync = False
        try:
            if lastOP == "F":  # or lastCommand[0] == "X":
                if (int(USAvgDistances[0]) < USThresholds[0]) or \
                        (int(USAvgDistances[1]) < USThresholds[1] ) or \
                        (int(USAvgDistances[2]) < USThresholds[1]):
                    print ("\t" + self.name + ": FRONT TOO CLOSE. STOPPPPP!!!")
                    obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[0]))
                           + ", " + str(int(USAvgDistances[1])) + ", " + str(int(USAvgDistances[0])))
                else:
                    obstruction = False
            elif lastOP == "B":  # or lastCommand[0] == "X":
                if int(USAvgDistances[5]) < USThresholds[2] :
                    print ("\t" + self.name + ": BACK TOO CLOSE. STOPPPPP!!!")
                    obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[5])))
                else:
                    obstruction = False
            elif lastOP == "L":
                if int(USAvgDistances[3]) < USThresholds[2]:
                    print ("\t" + self.name + ": LEFT SIDE TOO CLOSE. STOPPPPP!!!")
                    obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[3])))
                else:
                    obstruction = False
            elif lastOP == "R":
                if int(USAvgDistances[4]) < USThresholds[2]:
                    print ("\t" + self.name + ": RIGHT SIDE TOO CLOSE. STOPPPPP!!!")
                    obstruction = True
                    print ("\t" + self.name + " Avg Vector - " + str(int(USAvgDistances[4])))
                else:
                    obstruction = False

            threadSync = True
        except Exception as e:
            self.errorCount += 1
            print ("\t" + self.name + ": ERROR in US_safety_interrupt - " + str(e))


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


print ("Starting system...")
sysRunning = True

# setup serial connection to motor controller
print("Trying to connect to serial devices")

#Setup serial conn to motor controller
dataInput = raw_input("Connect to motor Controller... ? (Y/N)")
if dataInput == "y":
    dataInput = ""
    print("Trying to connect to motor controller")
    try:
        MotorConn = serial.Serial('/dev/ttyACM0', 19200)
        motorConnected = True
        print ('Connected to serial port /dev/ttyACM0')
        serialThread = motorDataThread(1, "#Serial com thread")
        serialThread.start()
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM0')
        motorConnected = False

#Setup serial conn to US controller
dataInput = raw_input("Connect Ultrasonic Controller... ? (Y/N)")
if dataInput == "y":
    dataInput = ""
    print("Trying to connect to Ultrasonic controller")
    try:
        USConn = serial.Serial('/dev/ttyACM1', 19200)
        USConnected = True
        print ('Connected to serial port /dev/ttyACM1')
        USthread = usDataThread(2, "#US data thread")
        USthread.start()
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM1')
        USConnected = True

#chose input method
dataInput = raw_input("Local Comms...? (y/n)")
if dataInput == "n":
    localCtrl = False
    # set the server address and port
    print("Setting up sockets...")
    HOST = get_ip_address('wlan0')  # socket.gethostbyname(socket.gethostname()) #socket.gethostname()
    PORT = 5002

    # create a socket to establish a server
    print("Binding the socket...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))

    # listen to incoming connections on PORT
    print 'Socket opened at ', HOST, 'listening to port ', PORT, '\n'
    s.listen(1)

#Main Control Loop
while sysRunning:
    if not localCtrl:
        # for each connection received create a tunnel to the client
        print ("ready for a new client to connect...")
        clientConnection, address = s.accept()
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
            dataInput = raw_input("Please Enter Commands on local terminal...")

        if dataInput == "e":
            cmdExpecting = False
            break
        elif dataInput == "q":
            cmdExpecting = False
            sysRunning = False
        else:
            print ("Client says - " + dataInput)
            if len(dataInput) > 0:
                if dataInput[0] == "f" or dataInput[0] == "b" or dataInput[0] == "r" \
                        or dataInput[0] == "l" or dataInput[0] == "x" or dataInput[0] == "m":
                    print ("Valid Command")
                    dataReady = True
                    lastOP = dataInput[0]  # [1:len(dataInput)]
                    #commandToTrans()
                    threadSync = False
                    speedVector = cmdToSpeeds(dataInput)
                    threadSync = True

                    if lastOP == "M":
                        print ("MANUAL OVERRIDE!\n")

                elif dataInput[0] == "s" :
                    if not safetyOn:
                        dataInput = raw_input("Do you solemnly swear you're up to no good!?..")
                        if dataInput[0] == "Y":
                            print ("May the force be with you... you are going to need it...")
                            print ("(just don't tell Ms Webster about it... >,< )")
                            safetyOn = False
                    else:
                        safetyOn = True
                        print ("Mischief managed ;)")
                    dataInput = ""
                    dataReady = False
            else:
                dataReady = False
                print ("Invalid Command")
        dataInput = ""
        print ("")

    if not localCtrl and clientConnection:
        # shut down the server
        clientConnection.close()
        print ("client at " + str(address) + " closed the connection ")
    else:
        print ("Use q to shutdown system... ")
        print ("Looping back to the start...")

exitFlag = 1
if not localCtrl:
    print ("Shutting down the server at " + HOST + "...")
    s.close()
else:
    print ("Shutting Down")
