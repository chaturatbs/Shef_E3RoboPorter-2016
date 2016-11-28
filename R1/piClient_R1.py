
import socket
import serial
import fcntl #linuz specific (keep note)
import struct
import time

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
porterMode = 0

#create a socket to establish a server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

#listen to incoming connections on PORT
print 'socket opened at ', HOST, 'listening to port ', PORT, '\n'
s.listen(1)

#setup serial connection to motor controller
try:
    MotorConn = serial.Serial('/dev/ttyACM0') #check this
except Exception as e:
    print ('unable to establish serial comms')
#motorConn.baudrate = 9600
#motorConn.port = 'serial'

while True:
    #for each connection received create a tunnel to the client
    print ("ready for a new client to connect...")
    clientConnection, address = s.accept()
    print 'Connected by', address

    #send welcome message
    clientConnection.send('connection ack')
    dataInput = clientConnection.recv(1024)
    porterMode = dataInput
    print ("porterMode set as " + porterMode)
    clientConnection.send("0")

    while True:
        #ready to move
        dataInput = clientConnection.recv(1024)
        clientConnection.send('1')
        print ("Navigator says " + dataInput)
        print ("Moving to node " + dataInput)
        time.sleep(3)
        print ("Successfully Moved to node " + dataInput)
        clientConnection.send('0')

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
