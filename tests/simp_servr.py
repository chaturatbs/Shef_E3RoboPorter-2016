#
#Module name - simp_servr
#Module Description -
#       A socket server using python socket modules.
#       Used to recieve data from a client over a network connection (wifi/ethernet)
#
#Author     - C. Samarakoon
#Created    - 16/10/2016
#Modified   - 18/10/2016
#

import socket
import serial


#set the server address and port
HOST= "127.0.0.1" #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT=5002
sonicSensorStatus = ""

#create a socket to establish a server
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))

#listen to incoming connections on PORT
print 'socket opened at ', HOST, 'listening to port ', PORT, '\n'   
s.listen(1)

while True:
    #for each connection recieved create a tunnel to the client
	connection, address = s.accept()
	print 'Connected by', address
    #send welcome message
	connection.send('connection ack')
	connection.close()

#sensor fetch function

#motor command send fn

#motor state get fn

motorConn = serial.Serial('/dev/USB0') #check this
print(motorConn.name)
motorConn.write(b'Hello there')
motorConn.write()
motorConn.readline()

sensorConn = serial.Serial('/dev/USB0')
sonicSensorStatus = sensorConn.readline().split("'") #data thats sent must me terminated by "\n"