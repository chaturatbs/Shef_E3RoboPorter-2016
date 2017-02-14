#based on the logging example on the python documentation

import pickle
import logging
import logging.handlers
import SocketServer
import struct
import time
import random

# class LogRecordStreamHandler(SocketServer.StreamRequestHandler):
#     """Handler for a streaming logging request.
#
#     This basically logs the record using whatever logging policy is
#     configured locally.
#     """
#
#     def handle(self):
#         """
#         Handle multiple requests - each expected to be a 4-byte length,
#         followed by the LogRecord in pickle format. Logs the record
#         according to whatever policy is configured locally.
#         """
#         while True:
#             chunk = self.connection.recv(4)
#             if len(chunk) < 4:
#                 break
#             slen = struct.unpack('>L', chunk)[0]
#             chunk = self.connection.recv(slen)
#             while len(chunk) < slen:
#                 chunk = chunk + self.connection.recv(slen - len(chunk))
#             obj = self.unPickle(chunk)
#             record = logging.makeLogRecord(obj)
#             self.handleLogRecord(record)
#
#     def unPickle(self, data):
#         return pickle.loads(data)
#
#     def handleLogRecord(self, record):
#         # if a name is specified, we use the named logger rather than the one
#         # implied by the record.
#         if self.server.logname is not None:
#             name = self.server.logname
#         else:
#             name = record.name
#         logger = logging.getLogger(name)
#         # N.B. EVERY record gets logged. This is because Logger.handle
#         # is normally called AFTER logger-level filtering. If you want
#         # to do filtering, do it at the client end to save wasting
#         # cycles and network bandwidth!
#         logger.handle(record)
#
# class LogRecordSocketReceiver(SocketServer.ThreadingTCPServer):
#     """
#     Simple TCP socket-based logging receiver suitable for testing.
#     """
#
#     allow_reuse_address = 1
#
#     def __init__(self, host='localhost',
#                  port=logging.handlers.DEFAULT_TCP_LOGGING_PORT,
#                  handler=LogRecordStreamHandler):
#         SocketServer.ThreadingTCPServer.__init__(self, (host, port), handler)
#         self.abort = 0
#         self.timeout = 1
#         self.logname = None
#
#     def serve_until_stopped(self):
#         import select
#         abort = 0
#         while not abort:
#             rd, wr, ex = select.select([self.socket.fileno()],
#                                        [], [],
#                                        self.timeout)
#             if rd:
#                 self.handle_request()
#             abort = self.abort
#
# def main():
#     logging.basicConfig(
#         format='%(relativeCreated)5d %(name)-15s %(levelname)-8s %(message)s')
#     tcpserver = LogRecordSocketReceiver()
#     print('About to start TCP server...')
#     tcpserver.serve_until_stopped()
#
# if __name__ == '__main__':
#     main()

import socket
import serial
import sys
from msvcrt import getch

#define variables for speed/angle/direction

#set the server address and port
HOST = "192.168.20.53" #raw_input("Please enter the server address: ") #"192.168.137.242" #"192.168.137.154"  #input("Enter the server address to connect to (default port is 5002) - ") #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
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

try:
    viewerCon = serial.Serial('COM7', 19200)
    viewerCon.open()
    vCon = True
except Exception as e:
    print(str(e))

#while the user doesnt stop communication using "esc"...
while True:
    data = s.recv(1024)
    data.rstrip("\n")
    #print (data)

    try:
        viewerCon.write(data)
    except Exception as e:
        print(str(e))
        try:
            print ("trying to connect to viewer")
            viewerCon = serial.Serial('COM7', 19200)
            viewerCon.open()

        except Exception as e:
            print(str(e))

s.close()