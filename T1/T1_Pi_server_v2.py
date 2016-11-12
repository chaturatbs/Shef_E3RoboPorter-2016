#
#Module name - T1_Pi_server
#Module Description -
#       A socket server using python socket modules.
#       Used to recieve data from a client over a network connection (wifi/ethernet)
#       This module will take recieve data from the client terminal and send appropriate commands to the motor Arduino
#
#Author     - C. Samarakoon
#Created    - 18/10/2016
#Modified   - 09/11/2016
#

import socket
import serial
import fcntl #linux specific (keep note)
import struct
import threading
import time

exitFlag = 0


def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Resolving ip address")
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])


class multiThreading (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
    def run(self):
        print "Starting " + self.name
        send_serial_data(self.name)
        print "Exiting " + self.name


def send_serial_data(threadName):

    while not exitFlag:
        try:
            print("Instructing to go at " + lastCommand)
            motorConn.write(lastCommand)

            print("Successfully sent...")
            #print ("Motor says - " + motorConn.readline())
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
        time.sleep(1.5)


#set the server address and port
print("Setting up sockets...")
HOST =  get_ip_address('wlan0') #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5002
global lastCommand
lastCommand= ""
dataInput = ""
sonicSensorStatus = []



#create a socket to establish a server
print("Binding the socket...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

#listen to incoming connections on PORT
print 'Socket opened at ', HOST, 'listening to port ', PORT, '\n'
s.listen(1)

global serialConnected
serialConnected = False
#setup serial connection to motor controller
print("Trying to connect to serial devices")
try:
    global motorConn
    motorConn = serial.Serial('/dev/ttyACM0', 19200) #check this
    serialConnected = True
    print ('Connected to serial port /dev/ttyACM0')
except Exception as e:
    print ('Unable to establish serial comms to port /dev/ttyACM0')

serialThread = multiThreading(1, "serial com thread")
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
            lastCommand = dataInput
            # if dataInput[0] == "#":
            #     print ("Valid Command")
            #     lastCommand = dataInput[1:len(dataInput)]
            # else:
            #     print ("Invalid Command")

        print ("")
    #shut down the server
    clientConnection.close()
    print ("client at " + str(address) + " closed the connection ")
    if dataInput == "q":
        print ("Shutting down the server at " + HOST + "...")
        exitFlag = 1
        s.close()
        break

