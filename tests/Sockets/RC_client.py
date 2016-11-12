#
#Module name - RC_client
#Module Description -
#       A socket client using python socket modules.
#       Used to send data from a PC terminal to a socket server through a network connection (wifi/ethernet)
#       Main purpose of this module is to operate the RoboPorter as an RC car for testing purposes
#
#Author     - C. Samarakoon
#Created    - 18/10/2016
#Modified   - 19/10/2016
#

import socket
import sys
from msvcrt import getch

#define variables for speed/angle/direction
direction = ""
motion = 0 #holds either required speed or angle.
angle = 0

motionMax = 100
motionMin = -100
motionDelta = 10
angleMax = 180
angleMin = -180
angleDelta = 10

#set the server address and port

HOST = "192.168.137.242"  #input("Enter the server address to connect to (default port is 5002) - ") #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
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

# while usInput != :
#     usInput = ""
#     print "what do you want to send?"
#     # get user input and send to the server
#     usInput = sys.stdin.readline()
#     s.send(usInput)

#while the user doesnt stop communication using "esc"...
while True:
    keystroke = ord(getch())
    if keystroke == 27: #if ESC break the loop
        print ("closing the connection to " + HOST + "...")
        s.send("e")
        break
    if keystroke == ord('q'):
        print ("Instructing the server at " + HOST + " to shut down...")
        s.send("q")
        print ("closing the connection to " + HOST + "...")
        break
    if keystroke == ord('r'): #r key for reset
        print ("reset values")
        motion = 0
        angle = 0
    elif keystroke == 224:  # Detect special keys
        keystroke = ord(getch())
        if keystroke == 80:  # Down arrow
            motion -= motionDelta
            if motion < motionMin:
                motion = motionMin
            #print("you said down")
        elif keystroke == 72:  # Up arrow
            motion += motionDelta
            if motion > motionMax:
                motion = motionMax
            #print("you said up")
        elif keystroke == 75:  # Left arrow pressed
            angle += angleDelta
            if angle > angleMax:
                angle = angleMax
            #print("you said left")
        elif keystroke == 77:  # right arrow pressed
            angle -= angleDelta
            if angle < angleMin:
                angle = angleMin
            #print("you said right")
    else:
        print("please enter a valid keystroke")
        continue
    print("motion = " + str(motion) + " angle = " + str(angle))
    #send data to server
    s.send(str(motion) + "," + str(angle))

#close connection once transmissions are done.
s.close()
