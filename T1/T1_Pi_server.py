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
import fcntl #linuz specific (keep note)
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("Resolving ip address")
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

#set the server address and port
print("Setting up sockets...")
HOST = get_ip_address('wlan0') #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5002

dataInput = ""
sonicSensorStatus = []

#create a socket to establish a server
print("Binding the socket...")
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

#listen to incoming connections on PORT
print 'Socket opened at ', HOST, 'listening to port ', PORT, '\n'
s.listen(1)

serialConnected = False
#setup serial connection to motor controller
print("Trying to connect to serial devices")
try:
    motorConn = serial.Serial('/dev/ttyACM0', 19200) #check this
    serialConnected = True
    print ('Connected to serial port /dev/ttyACM0')
except Exception as e:
    print ('Unable to establish serial comms to port /dev/ttyACM0')

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
            try:
                if not serialConnected:
                    print ("Serial port is closed")
                    try:
                        print ("Trying to open serial port")
                        motorConn.open()
                        serialConnected = True
                    except:
                        print("Serial port couldn't be opened :( ")
            except Exception as e:
                print ("No serial Comms... Looping back to listening mode")

            if serialConnected:
                print ("serial port is open")
                if (dataInput[0] == "F" or dataInput[0] == "B"): #and 0 < dataInput[1:len(dataInput)] < 1000:
                    if dataInput[0] == "F":
                        print("instructed to move FORWARDS by " + dataInput[1:len(dataInput)] + " cm")
                        motorConn.write("#" + dataInput)
                    if dataInput[0] == "B":
                        print("instructed to move BACKWARDS " + dataInput[1:len(dataInput)] + " cm")
                        motorConn.write("#" +dataInput)
                    print (motorConn.readline())
                elif (dataInput[0] == "L" or dataInput[0] == "R"): #and 0 < dataInput[1:len(dataInput)] < 180:
                    if dataInput[0] == "L":
                        print("instructed to ROTATE LEFT by " + dataInput[1:len(dataInput)] + " degrees")
                        motorConn.write("#" +dataInput)
                    if dataInput[0] == "R":
                        print("instructed to ROTATE RIGHT " + dataInput[1:len(dataInput)] + " degrees")
                        motorConn.write("#" + dataInput)
                    print (motorConn.readline())
                else:
                    print("instructed to go at " + dataInput)
                    motorConn.write(dataInput)
                    print ("motor says - " + str(motorConn.readline()))
        print ("")
    #shut down the server
    clientConnection.close()
    print ("client at " + str(address) + " closed the connection ")
    if dataInput == "q":
        print ("Shutting down the server at " + HOST + "...")
        s.close()
        break

