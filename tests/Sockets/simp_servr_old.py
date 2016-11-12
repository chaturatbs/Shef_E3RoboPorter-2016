import socket
import fcntl
import struct

def get_ip_address(ifname):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    return socket.inet_ntoa(fcntl.ioctl(
        s.fileno(),
        0x8915,  # SIOCGIFADDR
        struct.pack('256s', ifname[:15])
    )[20:24])

HOST = get_ip_address('wlan0')
PORT = 5002

s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind((HOST, PORT))
print 'socket opened at ', HOST, 'listening to port ', PORT, '\n'   
s.listen(1)
print 'waiting for connection...'
while 1:
	rx = ""
	connection, address = s.accept()
	print 'Connected by', address
	connection.send('connection ack')
	while rx != "exit\n":
		rx = connection.recv(1024)
		print (rx)
	if rx == "exit\n":
		print 'connection closing'
		connection.close()
		break
