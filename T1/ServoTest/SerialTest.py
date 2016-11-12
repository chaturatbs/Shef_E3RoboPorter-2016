import serial
import time

serialConnected = False
usInput = raw_input("input control command - ")

while True:
    if not serialConnected:
        # setup serial connection to motor controller
        print("Trying to connect to serial devices")
        try:
            motorConn = serial.Serial('COM4', 19200)  # EDIT THIS LINE
            serialConnected = True
            print ('Connected to serial port COM4')
        except Exception as e:
            print ('Unable to establish serial comms to port /dev/ttyACM0')
    else:
        print("Instructing to go at " + usInput)
        motorConn.write(usInput)

    time.sleep(1.5) #AND THIS LINE
