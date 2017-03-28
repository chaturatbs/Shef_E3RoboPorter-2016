#based on the logging example on the python documentation

import pickle
import logging
import logging.handlers
import SocketServer
import struct
import time
import random

import socket
import serial
import sys
from msvcrt import getch

import csv
from datetime import datetime

#define variables for speed/angle/direction

#set the server address and port
HOST = "192.168.30.94" #raw_input("Please enter the server address: ") #"192.168.137.242" #"192.168.137.154"  #input("Enter the server address to connect to (default port is 5002) - ") #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5003

#create a socket to connect to the server
s = socket.socket()
#connect to the server at HOST through PORT
print 'Trying to connect to ', HOST, 'at port ', PORT
s.connect((HOST, PORT))

#if connected (add error checking)

#recieve welcome message
print (("Server says - " + s.recv(1024)))

#initialise user input buffer and notify the server (for debugging)
usInput = ""
s.send("S1")
vCon = False
logToFile = False

try:
    viewerCon = serial.Serial('COM7', 19200)
    viewerCon.open()
    vCon = True
except Exception as e:
    print(str(e))

if logToFile:
    dt = datetime.now()
    fileName = "Logfile - " + dt.strftime("%Y_%m_%d-%H%M") + ".csv"
    logFile = open(fileName, 'wb')
    spamwriter = csv.writer(logFile, delimiter=' ',
                            quotechar='|', quoting=csv.QUOTE_MINIMAL)

#while the user doesnt stop communication using "esc"...

while True:
    data = s.recv(1024)
    data.rstrip("\n")
    #print (data)
    try:
        if logToFile:
            spamwriter.writerow(data)
        viewerCon.write(data)
        print data

    except Exception as e:
        print(str(e))
        try:
            print ("trying to connect to viewer")
            viewerCon = serial.Serial('COM7', 19200)
            viewerCon.open()

        except Exception as e:
            print(str(e))

s.close()
