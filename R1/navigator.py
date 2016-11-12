
#Module name - Navigator
#Module Description -
#
#
#Author     - C. Samarakoon
#Created    - 01/11/2016
#Modified   - 01/11/2016
#

from collections import deque
import socket

def directMode():
    print "zero"


def nodeListMode():
    print "one"


def directNodeMode():
    print "one"


def modeErr ():
    print "please choose a valid mode"


def opModeSwitch(mode):
    switcher = {
        1: directMode,
        2: nodeListMode,
        3: directNodeMode,
    }
    # Get the function from switcher dictionary
    func = switcher.get(mode, modeErr)
    # Execute the function
    return func()


def opModeChoice():
    # print options list
    print ("1: Direct Command Control")
    print ("2: Node list Control")
    print ("3: Direct Node Control")

    userInput = input("Please Choose an operation mode: ")
    opModeSwitch(int(userInput))


def setupClientConn():
    HOST = raw_input("Please enter the server address: ")  # "192.168.137.242" #"192.168.137.154"  #input("Enter the server address to connect to (default port is 5002) - ") #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
    PORT = 5002
    # create a socket to connect to the server
    s = socket.socket()

    # connect to the server at HOST through PORT
    try:
        print 'Trying to connect to ', HOST, 'at port ', PORT
        s.connect((HOST, PORT))
    except Exception as e:
        print("Connection to %s:%d resulted in an exception - %s" % (HOST, PORT, e))
    finally:
        s.close()

#define a node dictionary
nodeDict = {'n1': [1, 1],
            'n2': [3, 2],
            'n3': [1, 3],
            'n4': [1, 4],
            'n5': [1, 5]}

# define a node list, node name and location
nodeList = deque(['n1', 'n2', 'n4'])
lastSuccessNode = None
currentNode = None
porterStatus = 0
clientConnStat = 0
porterMode = 1

#connect to porter
#set the server address and port
HOST = raw_input("Please enter the server address: ")  # "192.168.137.242" #"192.168.137.154"  #input("Enter the server address to connect to (default port is 5002) - ") #socket.gethostbyname(socket.gethostname()) #socket.gethostname()
PORT = 5002
# create a socket to connect to the server
s = socket.socket()

# connect to the server at HOST through PORT
try:
    print 'Trying to connect to ', HOST, 'at port ', PORT
    s.connect((HOST, PORT))
    print ("Server says - " + s.recv(1024))
    #set porter mode
    s.send(str(porterMode))
    porterStatus = s.recv(1024)
    while porterStatus != "3":
        if (porterStatus == "0"):
            #send node
            try:
                currentNode = nodeList.popleft()
                print("currentNode is " + currentNode)
                #str1 = ', '.join(nodeDict[currentNode])
                print ("data to send = " + currentNode)
                s.send(currentNode)
                porterStatus = "1"
            except Exception as e:
                print("data action resulted in an exception - %s" % (HOST, PORT, e))

        print ("waiting for porter to move to node " + currentNode)
        while porterStatus == "1":
            porterStatus = s.recv(1024)
        if porterStatus == "0":
            print ("Successfully moved to node " + currentNode)
            lastSuccessNode = currentNode
    if porterStatus == "3":
        print("success!")

except Exception as e:
    print("Connection to %s:%d resulted in an exception - %s" % (HOST, PORT, e))
finally:
    s.close()


#send nodes
    #if ready to move, send next node

    #wait until motion completed
#repeat

#exit on error