#
#Module name - T4_Pi_server
#Module Description -
#       A socket server using python socket modules.
#       Used to recieve data from a client over a network connection (wifi/ethernet)
#       This module will take recieve data from the client terminal and send appropriate commands to the motor Arduino
#
#Author     - C. Samarakoon
#Created    - 18/10/2016
#Modified   - 16/11/2016
#

import socket
import serial
import fcntl #linux specific (keep note)
import struct
import threading
import time
import numpy

global lastCommand
lastCommand= ""
global motorConnected
motorConnected = False
global USConnected
USConnected = False
global motorConn
global USConn
global dataReady
dataReady = False
global USAvgDistances
USAvgDistances = [0.,0.,0.,0.,0.,0.]
global obstruction
obstruction = False

dataInput = ""
exitFlag = 0


global tMat
global theta
global porterLocation
global porterOrientation

theta = 0.0
tMat = numpy.array([0, 0])
porterLocation = numpy.array([0, 0, 0])
porterOrientation = 0.0 # from starting heading
localCtrl = True
sysRunning = False
cmdExpecting = False

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Resolving ip address")
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

class multiThreadBase (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.avgRuntime = 0.
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopDebugInterval = 10


    def loopStartFlag (self):
        self.startTime = time.localtime()
        pass

    def loopEndFlag (self):
        self.startTime = time.localtime()
        pass

    def loopRunTime (self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime)/self.loopDebugInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopDebugInterval :
            print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            self.avgCounter = 0

class SerialThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        #self.dataReady = 0
        self.actionState = 0
        self.debug = False
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopDebugInterval = 10
        self.avgRuntime = 0

    def run(self):
        print "Starting " + self.name
        while not exitFlag:
            self.loopStartFlag()
            if obstruction :
                print (self.name + " OBSTRUCTION! :( ")
                self.send_serial_data("X")

            elif dataReady:
                print (self.name + " data Ready")
                #if self.actionState == 1:
                try:
                    #if freeToMove == False:
                        #print(self.name + "Obstruction.. cant move...")
                    print (self.name + " trying to send data")
                    self.send_serial_data(lastCommand)
                except Exception as e:
                    print (self.name + ": ERROR - " + str(e))
                    try:
                        print (self.name + ": Trying to open serial port")
                        motorConn.open()
                        #serialConnected = True
                    except Exception as e:
                        print(self.name + ": ERROR - Motor port couldn't be opened :( : " + str(e))
                    finally:
                        print (self.name + ": No Motor Comms... Looping back to listening mode")
            self.read_serial_data()
            if self.debug:
                time.sleep(1)
            else:
                time.sleep(0.001)
            #self.loopEndFlag()
            #self.loopRunTime()
        print "Exiting " + self.name

    def send_serial_data(self, sendCommand):
        # while not exitFlag:
        global dataReady

        print(self.name + ": Instructing to move as - " + sendCommand)
        motorConn.write(sendCommand)
        #if lastCommand[0] != "X":
        dataReady = False
        if self.debug:
            print(self.name + ": Successfully sent...")
        self.actionState = 4

    def read_serial_data(self):
        motorInput = motorConn.readline()
        if self.debug:
            print (self.name + ": motor says " + motorInput)
        if motorInput == "3\r\n": #recieved
            self.actionState = 3
            print (self.name + " - Command successfully received by motor...")
        elif motorInput == "2\r\n":
            if self.debug:
                print (self.name + "- Motor actuating command...")
            self.actionState = 2
        elif motorInput == "1\r\n":
            if self.debug:
                print (self.name + "- Motor actuation finished... ready to accept new data :) ")
            self.actionState = 1
        elif motorInput == "5\r\n":
            print (self.name + "- ERROR TRYING TO EXECUTE COMMAND :( ...")
            self.actionState = 5

    def loopStartFlag (self):
        self.startTime = time.localtime()
        pass

    def loopEndFlag (self):
        self.startTime = time.localtime()
        pass

    def loopRunTime (self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime)/self.loopDebugInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopDebugInterval :
            print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            self.avgCounter = 0

class usDataThread (multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata = [0.,0.,0.,0.,0.,0.]
        self.inputBuf = ""
        self.errorCount = 0
        self.debug = False

    def run(self):
        global lastCommand
        print "Starting " + self.name
        while not exitFlag:
            #self.loopStartFlag()
            try:
                self.getUSvector()
                self.mAverage(5)
            except Exception as e:
                self.errorCount += 1
                print (self.name + ": ERROR - " + str(e))
                try:
                    if self.debug:
                        print (self.name + ":Trying to open serial port")
                    USConn.open()
                    serialConnected = True
                except Exception as e:
                    self.errorCount += 1
                    print(self.name + ": ERROR - Ultrasonic port couldn't be opened :( : " + str(e))
                finally:
                    print (self.name + ": No Ultrasonic comms... Halting... ")
                    lastCommand = "X"

            if self.errorCount > 3:
                print ("Consecutive Exceptions... pausing execution...")
                self.errorCount = 0
                time.sleep(3)

            self.US_safety_interrupt()
            if self.debug:
                time.sleep(1)
            else:
                time.sleep(0.01)

            #USConn.flush()
            #self.loopEndFlag()
            #self.loopRunTime()
        print "Exiting " + self.name

    def getUSvector(self):
        if self.debug:
            print (self.name + " inside getUSVector")
        try:
            if self.debug:
                print (self.name + ": trying to get US data")
            self.inputBuf = USConn.readline()
            self.inputBuf.rstrip("\n\r")
            self.rawUSdata = self.inputBuf.split(",")
            #print (self.name + " US says - " + self.inputBuf)
            #self.rawUSdata[len(self.rawUSdata)] = str(self.rawUSdata[self.rawUSdata[len(self.rawUSdata)]])[0:self.rawUSdata[len(self.rawUSdata)-2]]
            if self.debug:
                print (self.name + " US Raw Vector - " + str(self.rawUSdata))
        except Exception as e:
            self.errorCount += 1
            print (self.name + ": ERROR trying to get US data - " + str(e))

    def mAverage(self, n):
        global USAvgDistances
        #print (self.name + " inside mAverage")
        i = 0
        for i in range(0, len(USAvgDistances)):
            USAvgDistances[i] += (int(self.rawUSdata[i]) - USAvgDistances[i])/n
        if self.debug:
            print (self.name + " Avg Vector - " + str(USAvgDistances))

    def US_safety_interrupt(self):
        global USAvgDistances
        global lastCommand
        global dataReady
        global obstruction
        #print (self.name + " inside US-Interrupt")
        try:
            if lastCommand[0] == "F": #or lastCommand[0] == "X":
                if (int(USAvgDistances[0]) < 100) or (int(USAvgDistances[1]) < 70) or (int(USAvgDistances[2]) < 70):
                    print (self.name + ": FRONT TOO CLOSE. STOPPPPP!!!")
                    #dataReady = True
                    obstruction = True
                    print (self.name + " Avg Vector - " + str(USAvgDistances))
                    #lastCommand = "X"
                    #print (self.name + ": Sending Command...")
                    #motorConn.write(lastCommand)
                    #time.sleep(1)
                else:
                    obstruction = False
            elif lastCommand[0] == "B": #or lastCommand[0] == "X":
                if int(USAvgDistances[5]) < 50:
                    print (self.name + ": BACK TOO CLOSE. STOPPPPP!!!")
                    #dataReady = True
                    obstruction = True
                    print (self.name + " Avg Vector - " + str(USAvgDistances))
                    #lastCommand = "X"
                    # print (self.name + ": Sending Command...")
                    # motorConn.write(lastCommand)
                    # time.sleep(1)
                else :
                    obstruction = False

        except Exception as e:
            self.errorCount += 1
            print (self.name + ": ERROR in US_safety_interrupt - " + str(e))


def simpTransform():
    pass
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

#setup serial connection to motor controller
print("Trying to connect to serial devices")

dataInput = raw_input("Connect to motor Controller... ? (Y/N)")
if dataInput == "y":
    dataInput = ""
    print("Trying to connect to motor controller")
    try:
        motorConn = serial.Serial('/dev/ttyACM0', 19200)
        motorConnected = True
        print ('Connected to serial port /dev/ttyACM0')
        serialThread = SerialThread(1, "#Serial com thread")
        serialThread.start()
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM0')


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


dataInput = raw_input("Local Comms...? (y/n)")
if dataInput == "n":
    localCtrl = False
    #set the server address and port
    print("Setting up sockets...")
    HOST = get_ip_address('wlan0') #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
    PORT = 5002

    #create a socket to establish a server
    print("Binding the socket...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))

    #listen to incoming connections on PORT
    print 'Socket opened at ', HOST, 'listening to port ', PORT, '\n'
    s.listen(1)


while sysRunning:
    if not localCtrl:
        #for each connection received create a tunnel to the client
        print ("ready for a new client to connect...")
        clientConnection, address = s.accept()
        print 'Connected by', address

        #send welcome message
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
            if dataInput[0] == "F" or dataInput[0] == "B" or dataInput[0] == "R" or dataInput[0] == "L" or dataInput[0] == "X":
                print ("Valid Command")
                dataReady = True
                lastCommand = dataInput #[1:len(dataInput)]
                if lastCommand[0] != "X" :
                    commandToTrans()
                # else:
                #     motorConn.write(lastCommand)
                #     dataReady = False
            else:
                dataReady = False
                print ("Invalid Command")
        dataInput = ""
        print ("")

    if not localCtrl:
        #shut down the server
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
