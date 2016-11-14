#
#Module name - RC_server
#Module Description -
#       A socket server using python socket modules.
#       Used to recieve data from a client over a network connection (wifi/ethernet)
#       This module will take recieve data from the client terminal and send appropriate commands to the motor Arduino
#
#Author     - C. Samarakoon
#Created    - 18/10/2016
#Modified   - 13/11/2016
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
global serialConnected
serialConnected = False
global motorConn

dataInput = ""
exitFlag = 0
USAvgDistances = []

global tMat
global theta
global porterLocation
global porterOrientation

theta = 0.0
tMat = numpy.array([0, 0])
porterLocation = numpy.array([0, 0, 0])
porterOrientation = 0.0 # from starting heading

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
        self.dataReady = 0
        self.actionState = 0

    def run(self):
        print "Starting " + self.name
        while not exitFlag:
            if self.dataReady:
                if self.actionState == 0:
                    try:
                        self.send_serial_data()
                    except Exception as e:
                        print ("ERROR - " + str(e))
                        try:
                            print ("Trying to open serial port")
                            motorConn.open()
                            serialConnected = True
                        except Exception as e:
                            print("ERROR - Serial port couldn't be opened :( : " + str(e))
                        finally:
                            print ("No serial Comms... Looping back to listening mode")
            self.read_serial_data()
            time.sleep(1.5)
        print "Exiting " + self.name

    def send_serial_data(self):
        # while not exitFlag:
        print("Instructing to move as " + lastCommand)
        motorConn.write(lastCommand)
        print("Successfully sent...")
        self.actionState = 4

    def read_serial_data(self):
        motorInput = motorConn.readline()
        print ("motor says " + motorInput)
        if motorInput == "3": #recieved
            self.actionState = 3
            print ("Command successfully received by motor...")
        elif motorInput == "2":
            print ("Motor actuating command...")
            self.actionState = 2
        elif motorInput == "1":
            print ("Motor actuation finished...")
            self.actionState = 1
        elif motorInput == "5":
            print ("ERROR TRYING TO EXECUTE COMMAND :( ...")
            self.actionState = 5



class usDataThread (multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata = []
        self.inputBuf = ""

    def run(self):
        print "Starting " + self.name
        self.getUSvector()
        self.mAverage(5)
        print "Exiting " + self.name

    def getUSvector(self):
        pass

    def mAverage(self, n):
        i = 0
        for i in [0, 6]:
            USAvgDistances[i] = USAvgDistances[i] + (self.rawUSdata[i] - USAvgDistances[i])/n


def simpTransform():
    porterLocation = porterLocation + tMat
    porterOrientation = porterOrientation + theta


def commandToTrans():
    if lastCommand[0] == "F":
        tMat[0] = lastCommand[1:len(lastCommand)]
    elif lastCommand[0] == "B":
        tMat[1] = lastCommand[1:len(lastCommand)]
    elif lastCommand[0] == "L":
        theta = lastCommand[1:len(lastCommand)]
    elif lastCommand[0] == "R":
        theta = (-1) * lastCommand[1:len(lastCommand)]

#set the server address and port
print("Setting up sockets...")
HOST =  get_ip_address('wlan0') #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5002

#create a socket to establish a server
print("Binding the socket...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

#listen to incoming connections on PORT
print 'Socket opened at ', HOST, 'listening to port ', PORT, '\n'
s.listen(1)

#setup serial connection to motor controller
print("Trying to connect to serial devices")
try:
    motorConn = serial.Serial('/dev/ttyACM0', 19200) #check this
    serialConnected = True
    print ('Connected to serial port /dev/ttyACM0')
except Exception as e:
    print ('Unable to establish serial comms to port /dev/ttyACM0')

serialThread = SerialThread(1, "serial com thread")
serialThread.start()

while True:
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

    while True:
        dataInput = clientConnection.recv(1024)
        if dataInput == "e":
            break
        elif dataInput == "q":
            break
        else:
            print ("Client says - " + dataInput)
            #lastCommand = dataInput
            if dataInput[0] == "#":
                print ("Valid Command")
                lastCommand = dataInput[1:len(dataInput)]
                commandToTrans()
            else:
                print ("Invalid Command")

        print ("")
    #shut down the server
    clientConnection.close()
    print ("client at " + str(address) + " closed the connection ")
    if dataInput == "q":
        print ("Shutting down the server at " + HOST + "...")
        exitFlag = 1
        s.close()
        break
