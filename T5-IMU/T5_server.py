#
#Module name - T5_Pi_server
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
import fcntl #linux specific (keep note)
import struct
import threading
import time
import sys, getopt

sys.path.append('.')
import RTIMU
import os.path
import math

import MadgwickFilter

exitFlag = 0
USAvgDistances = []
global lastCommand
lastCommand= ""
dataInput = ""
global IMU_data


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

class IMU_Thread (multiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.rawUSdata = []
        self.inputBuf = ""
        self.SETTINGS_FILE = "RTIMULib"

        self.s = RTIMU.Settings(self.SETTINGS_FILE)
        self.imu = RTIMU.RTIMU(self.s)
        self.IMU_conStat = False
        self.poll_interval = 0.0

    def run(self):
        print "Starting " + self.name
        self.IMU_setup()
        if self.IMU_conStat:
            self.IMU_run()
        print "Exiting " + self.name

    def IMU_setup(self):

        print("Using settings file " + self.SETTINGS_FILE + ".ini")
        if not os.path.exists(self.SETTINGS_FILE + ".ini"):
            print("Settings file does not exist, will be created")

        print("IMU Name: " + self.imu.IMUName())

        if (not self.imu.IMUInit()):
            print("IMU Init Failed")
            self.IMU_conStat = False
            #sys.exit(1)
        else:
            print("IMU Init Succeeded")
            self.IMU_conStat = True

    def IMU_run (self):
        self.imu.setSlerpPower(0.02)
        self.imu.setGyroEnable(True)
        self.imu.setAccelEnable(True)
        self.imu.setCompassEnable(True)

        self.poll_interval = self.imu.IMUGetPollInterval()
        print("Recommended Poll Interval: %dmS\n" % self.poll_interval)

        while True:
            if self.imu.IMURead():
                # x, y, z = imu.getFusionData()
                # print("%f %f %f" % (x,y,z))
                IMU_data = self.imu.getIMUData()
                fusionPose = IMU_data["fusionPose"]
                #madData = MadgwickFilter.MadgwickAHRSupdate()
                print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),
                                             math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
                try:
                    clientConnection.send("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),
                                             math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
                except:
                    pass
                time.sleep(self.poll_interval * 1.0 / 1000.0)



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


newIMUstream = IMU_Thread(1, "IMU thread")
newIMUstream.start()

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

        print ("")
    #shut down the server
    clientConnection.close()
    print ("client at " + str(address) + " closed the connection ")
    if dataInput == "q":
        print ("Shutting down the server at " + HOST + "...")
        exitFlag = 1
        s.close()
        break
