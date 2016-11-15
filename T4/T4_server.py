#
#Module name - T4_Pi_server
#Module Description -
#       A socket server using python socket modules.
#       Used to recieve data from a client over a network connection (wifi/ethernet)
#       This module will take recieve data from the client terminal and send appropriate commands to the motor Arduino
#
#Author     - C. Samarakoon
#Created    - 18/10/2016
#Modified   - 14/11/2016
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

dataInput = ""
exitFlag = 0
USAvgDistances = [0.,0.,0.,0.,0.,0.]

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

class SerialThread (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.dataReady = 1
        self.actionState = 0

    def run(self):
        print "Starting " + self.name
        while not exitFlag:
            if self.dataReady:
                if self.actionState == 1:
                    try:
                        print ("trying to send data")
                        self.send_serial_data()
                    except Exception as e:
                        print (self.name + ": ERROR - " + str(e))
                        try:
                            print (self.name + ": Trying to open serial port")
                            motorConn.open()
                            serialConnected = True
                        except Exception as e:
                            print(self.name + ": ERROR - Motor port couldn't be opened :( : " + str(e))
                        finally:
                            print (self.name + ": No Motor Comms... Looping back to listening mode")
            self.read_serial_data()
            time.sleep(1.5)
        print "Exiting " + self.name

    def send_serial_data(self):
        # while not exitFlag:
        print(self.name + ": Instructing to move as - " + lastCommand)
        motorConn.write(lastCommand)
        print(self.name + ": Successfully sent...")
        self.actionState = 4

    def read_serial_data(self):
        motorInput = motorConn.readline()
        print (self.name + ": motor says " + motorInput)
        if motorInput == "3\r\n": #recieved
            self.actionState = 3
            print (self.name + " - Command successfully received by motor...")
        elif motorInput == "2\r\n":
            print (self.name + "- Motor actuating command...")
            self.actionState = 2
        elif motorInput == "1\r\n":
            print (self.name + "- Motor actuation finished... ready to accept new data :) ")
            self.actionState = 1
        elif motorInput == "5\r\n":
            print (self.name + "- ERROR TRYING TO EXECUTE COMMAND :( ...")
            self.actionState = 5


class usDataThread (multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata = [0.,0.,0.,0.,0.,0.]
        self.inputBuf = ""
        self.errorCount = 0

    def run(self):
        print "Starting " + self.name
        while not exitFlag:
            try:
                self.getUSvector()
                self.mAverage(5)
            except Exception as e:
                self.errorCount += 1
                print (self.name + ": ERROR - " + str(e))

                try:
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
            time.sleep(0.1)
        print "Exiting " + self.name

    def getUSvector(self):
        pass
        # try:
        #     print (self.name + ": trying to get US data")
        #     self.inputBuf = USConn.readline()
        #     self.rawUSdata = float(self.inputBuf.split(","))
        # except Exception as e:
        #     self.errorCount += 1
        #     print (self.name + ": ERROR trying to get US data - " + e)

    def mAverage(self, n):
        pass
        # i = 1
        # for i in [1, 6]:
        #     USAvgDistances[i] = USAvgDistances[i] + (self.rawUSdata[i] - USAvgDistances[i])/n

    def US_safety_interrupt(self):
        try:
            if USAvgDistances[1] < 30:
                motorConn.write("X")
        except Exception as e:
            self.errorCount += 1
            print (self.name + ": ERROR in US_safety_interrupt - " + e)


def simpTransform():
    pass
    # porterLocation = porterLocation + tMat
    # porterOrientation = porterOrientation + theta


def commandToTrans():
    if lastCommand[0] == "F":
        tMat[0] = lastCommand[1:len(lastCommand)]
    elif lastCommand[0] == "B":
        tMat[1] = lastCommand[1:len(lastCommand)]
    elif lastCommand[0] == "L":
        theta = lastCommand[1:len(lastCommand)]
    elif lastCommand[0] == "R":
        theta = (-1) * lastCommand[1:len(lastCommand)]


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
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM0')

    serialThread = SerialThread(1, "#Serial com thread")
    serialThread.start()


dataInput = raw_input("Connect Ultrasonic Controller... ? (Y/N)")
if dataInput == "y":
    dataInput = ""
    print("Trying to connect to Ultrasonic controller")
    try:
        USConn = serial.Serial('/dev/ttyACM1', 19200)
        USConnected = True
        print ('Connected to serial port /dev/ttyACM1')
    except Exception as e:
        print ('Unable to establish serial comms to port /dev/ttyACM1')

    USthread = usDataThread(2, "#US data thread")
    USthread.start()


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
            if dataInput[0] == "#":
                print ("Valid Command")
                lastCommand = dataInput[1:len(dataInput)]
                if not lastCommand[0] == "X":
                    commandToTrans()
                else:
                    motorConn.write(lastCommand)
            else:
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
print ("Shutting down the server at " + HOST + "...")
if not localCtrl:
    s.close()
