import os
import numpy as np
from vpLib import *
import logging
import threading
import datetime
import Queue
import time
#from matplotlib import pyplot as plt

grid_size = [0,0]
global cam
global vanish
global vpValid

threadQueue = []
fps = [0]
frameNumber = [0]

# plt.axis = [0,360,0,50]
# plt.ion()


class MultiThreadBase(threading.Thread): #Parent class for threading
    def __init__(self, threadID, name): #Class constructor
        threading.Thread.__init__(self) #Initiate thread
        self.threadID = threadID #Thread number
        self.name = name #Readable thread name

        #thread loop performance profiling
        self.avgRuntime = 0. #loop average runtime
        self.startTime = 0. #loop start time
        self.endTime = 0. #loop end time
        self.avgCounter = 0 #number of loops
        self.loopProfilingInterval = 10 #loop averaging interval
        self.profiling = False #profiling state. set true in to calculate run times globally.
            #if you want to profile a single thread, change this in the child thread
            #Note - the system will run much slower when profiling
            #STILL NOT FULLY IMPLEMENTED. DO NOT SET TO TRUE

    def loopStartFlag(self): #run at the start of the loop
        self.startTime = time.localtime() #record the start time of the loop

    def loopEndFlag(self):
        self.startTime = time.localtime()

    def loopRunTime(self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime) / self.loopProfilingInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopProfilingInterval:
            # print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            logging.debug("Avg loop Runtime - %s", str(self.avgRuntime))
            self.avgCounter = 0


class CameraThreadClass(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.pulseData = ["",""] #pulses counted by the motor controller
        self.vanishx = [0, 0, 0, 0, 0]
        self.vanishy = [0, 0, 0, 0, 0]
        self.vanishxVar = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.vanishyVar = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def run(self):
        global cam

        self.vpFromCAM()


    def mAverage(self, vpCoord, n):
        i = 0

        if len(vpCoord) == 2:
            for i in range(0, 2):
                vanish[i] += (int(vpCoord[i]) - vanish[i]) / n

        return vanish

    def medianFilter(self, vpCoord, n):
        i = 0
        #global vpValid

        vanish = [0.,0.]

        if len(vpCoord) == 2:
            self.vanishx[0] = self.vanishx[1]
            self.vanishx[1] = self.vanishx[2]
            self.vanishx[2] = self.vanishx[3]
            self.vanishx[3] = self.vanishx[4]
            self.vanishx[4] = vpCoord[0]

            self.vanishy[0] = self.vanishy[1]
            self.vanishy[1] = self.vanishy[2]
            self.vanishy[2] = self.vanishy[3]
            self.vanishy[3] = self.vanishy[4]
            self.vanishy[4] = vpCoord[1]

            sortedx = sorted(self.vanishx)
            sortedy = sorted(self.vanishy)

            vanish = (sortedx[2],sortedy[2])

        return vanish

    def varianceFilter(self, vpCoord, n):
        global vpValid
        i = 0

        for i in range(0, n - 1):
            self.vanishxVar[i] = self.vanishxVar[i + 1]
            self.vanishyVar[i] = self.vanishyVar[i + 1]

        self.vanishxVar[n - 1] = vpCoord[0]
        self.vanishyVar[n - 1] = vpCoord[1]

        medVar = np.var(self.vanishxVar), np.var(self.vanishyVar)

        if (medVar[0] > 150) or (medVar[1] > 150):
            vpValid = 0
        else:
            vpValid = 1

        return 0

    def vpFromCAM(self):
        global vanish

        # img2 = np.zeros((540, 960, 3), np.uint8)
        # img3 = np.zeros((540, 960, 3), np.uint8)
        capVid = cv2.VideoCapture(0)

        #capVid = cv2.VideoCapture('cor2small.mp4')  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        outvid = cv2.VideoWriter('output.avi', fourcc, 5.0, (640,480))

        if capVid.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
            print "error: capVid not accessed successfully\n\n"  # if not, print error message
            #logging.error("error: capWebcam not accessed successfully\n\n")
            os.system("pause")

        while cv2.waitKey(1) != 27 and capVid.isOpened():
            blnFrameReadSuccessfully, img = capVid.read()
            #origimg = img
            startTime = time.time()
            sumTime = 0
            outvid.write(img)

            print "Image Loaded: " + str(startTime)

            if len(frameNumber) > 0:
                frameNumber.append(frameNumber[len(frameNumber) - 1] + 1)
            else:
                frameNumber[0] = 1

            try: #try to find vanishing point
                hough_lines, startTime, sumTime = hough_transform(img, False, startTime)  #calculate hough lines

                if hough_lines: #if lines found
                    random_sample = getLineSample(hough_lines, 30)  # take a sample of 100 line
                    intersections = find_intersections(random_sample, img)  # Find intersections in the sample

                    duration = time.time() - startTime
                    print "Intersection Time :" + str(duration)
                    sumTime += duration
                    startTime = time.time()

                    if intersections:  # if intersections are found
                        grid_size[0] = img.shape[0] // 8 #set the grid size to be 20 by 20
                        grid_size[1] = img.shape[1] // 20
                        #find vanishing points
                        vanishing_point = vp_candidates(img, grid_size, intersections)
                        #returns the best cell

                        vanish2 = self.medianFilter(vanishing_point[0], 5)
                        x = self.varianceFilter(vanishing_point[0], 10)

                        if vpValid == 1:
                            cv2.circle(img, (vanish2[0], vanish2[1]), 5, (210, 255, 10), thickness=2)
                        else:
                            cv2.circle(img, (vanish2[0], vanish2[1]), 5, (10, 10, 255), thickness=2)

                cv2.imshow('vp Image', img)

                duration = time.time() - startTime
                print "Finish Time :" + str(duration)
                sumTime += duration
                startTime = time.time()

                expectedFPS = 1/sumTime

                print "Expected FPS: " + str(expectedFPS)

                fps.append(expectedFPS)
                # plt.plot(frameNumber, fps)
                # plt.pause(0.05)

                print "----------------------------------------------"

            except Exception as e:
                pass

        capVid.release()
        outvid.release()

        cv2.destroyAllWindows()


class PIDThreadClass(MultiThreadBase):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.tuning = (0.1,0.1,0)
        self.vanishsum = 0

    def run(self):
        threading.Timer(0.5, self.run).start()
        global vanish
        #self.vanishsum += vanish[0]
        #result = (self.tuning[0] * vanish[0]) + (self.tuning[1] * self.vanishsum)
        #print datetime.datetime.now(), result

if __name__ == '__main__':
    cam = False
    vanish = [0,0]
    vpValid = 0

    vpThread = CameraThreadClass(1,"vpThread")
    vpThread.start()
    threadQueue.append(vpThread)

    # PIDThread = PIDThreadClass(2,"PIDThread")
    # PIDThread.start()
    # threadQueue.append(PIDThread)

    # uInput = ""
    # while uInput != "q":
    #     uInput = input("press q to exit...")
    #

    for t in threadQueue:
        t.join()

    print "stopping..."
##end