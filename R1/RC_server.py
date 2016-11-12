#
#Module name - RC_server
#Module Description -
#       A socket server using python socket modules.
#       Used to recieve data from a client over a network connection (wifi/ethernet)
#       This module will take recieve data from the client terminal and send appropriate commands to the motor Arduino
#
#Author     - C. Samarakoon
#Created    - 18/10/2016
#Modified   - 18/10/2016
#

import socket
import serial
import fcntl #linuz specific (keep note)
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

#set the server address and port
HOST = get_ip_address('eth0') #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5002
sonicSensorStatus = []
dataInput = ""

#create a socket to establish a server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

#listen to incoming connections on PORT
print 'socket opened at ', HOST, 'listening to port ', PORT, '\n'
s.listen(1)

#setup serial connection to motor controller
motorConn = serial.Serial('/dev/ttyACM0') #check this
#motorConn.baudrate = 9600
#motorConn.port = 'serial'

while True:
    #for each connection received create a tunnel to the client
    print ("ready for a new client to connect...")
    clientConnection, address = s.accept()
    print 'Connected by', address

    #send welcome message
    clientConnection.send('connection ack')
    while True:
        dataInput = clientConnection.recv(1024)
        if dataInput == "e":
            break
        elif dataInput == "q":
            break
        else:
            print ("Client says - " + dataInput)
            #print ("Trying to open serial port")
            if not motorConn._isOpen:
                print ("serial port is closed")
                try:
                    print ("Trying to open serial port")
                    motorConn.open()
                except:
                    print("serial port couldnt be opened :( ")
            if motorConn._isOpen:
                print ("serial port is open")
                if (dataInput[0] == "F" or dataInput[0] == "B"): #and 0 < dataInput[1:len(dataInput)] < 1000:
                    if dataInput[0] == "F":
                        print("instructed to move FORWARDS by " + dataInput[1:len(dataInput)] + " cm")
                        motorConn.write("#" + dataInput)
                    if dataInput[0] == "B":
                        print("instructed to move BACKWARDS " + dataInput[1:len(dataInput)] + " cm")
                        motorConn.write(dataInput)
                    print (motorConn.readline())
                elif (dataInput[0] == "L" or dataInput[0] == "R"): #and 0 < dataInput[1:len(dataInput)] < 180:
                    if dataInput[0] == "L":
                        print("instructed to ROTATE LEFT by " + dataInput[1:len(dataInput)] + " degrees")
                        motorConn.write(dataInput)
                    if dataInput[0] == "R":
                        print("instructed to ROTATE RIGHT " + dataInput[1:len(dataInput)] + " degrees")
                        motorConn.write(dataInput)
                    print (motorConn.readline())

    #shut down the server
    clientConnection.close()
    print ("client at " + str(address) + " closed the connection ")
    if dataInput == "q":
        print ("Shutting down the server at " + HOST + "...")
        s.close()
        break

#sensor fetch function

#motor command send fn

#motor state get fn
#

#print(motorConn.name)
#if motorConn._isOpen:
  #  motorConn.write(b'Hello there')
  #  motorConn.write(dataInput)
# motorConn.readline()

# sensorConn = serial.Serial('/dev/USB0')
# sonicSensorStatus = sensorConn.readline().split(",") #data thats sent must me terminated by "\n"
