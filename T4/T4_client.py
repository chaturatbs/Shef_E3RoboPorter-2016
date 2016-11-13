#
#Module name - T4 PC_client
#Module Description -
#       A socket client using python socket modules.
#       Used to send data from a PC terminal to a socket server through a network connection (wifi/ethernet)
#
#
#Author     - C. Samarakoon
#Created    - 10/11/2016
#Modified   - 10/11/2016
#

import socket
import sys
from msvcrt import getch

#define variables for speed/angle/direction


#set the server address and port
HOST = raw_input("Please enter the server address: ") #"192.168.137.242" #"192.168.137.154"  #input("Enter the server address to connect to (default port is 5002) - ") #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5002

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
s.send("I am about to send data")


#while the user doesnt stop communication ...
while True:
    usInput = raw_input("input control command - ")

    if usInput == "exit": #if ESC break the loop
        print ("closing the connection to " + HOST + "...")
        s.send("e")
        break
    if usInput == "shutdown":
        print ("Instructing the server at " + HOST + " to shut down...")
        s.send("q")
        print ("closing the connection to " + HOST + "...")
        break
    inputBuf = s.recv(1024)
    #send data to server
    s.send(usInput)

#close connection once transmissions are done.
s.close()
