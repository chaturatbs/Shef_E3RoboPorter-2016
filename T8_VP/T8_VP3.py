import os
import numpy as np
from vpLib import *
import logging
import threading
import Queue

grid_size = [0,0]
global cam

threadQueue = []


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

    def run(self):
        global cam

        if cam:
            self.vpFromCam()
        else:
            self.vpFromImg()

    def vpFromCam(self):
        capWebcam = cv2.VideoCapture(0)  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam
        if capWebcam.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
            print "error: capWebcam not accessed successfully\n\n"  # if not, print error message
            #logging.error("error: capWebcam not accessed successfully\n\n")
            os.system("pause")

        while cv2.waitKey(1) != 27 and capWebcam.isOpened():
            blnFrameReadSuccessfully, img = capWebcam.read()

            try: #try to find vanishing point
                hough_lines = hough_transform(img, False)  #calculate hough lines
                if hough_lines: #if lines found
                    random_sample = getLineSample(hough_lines, 100)  # take a sample of 100 lines
                    intersections = find_intersections(random_sample, img)  # Find intersections in the sample
                    if intersections:  # if intersections are found
                        grid_size[0] = img.shape[0] // 20 #set the grid size to be 20 by 20
                        grid_size[1] = img.shape[1] // 20
                        #find vanishing points
                        vanishing_point = vp_candidates(img, grid_size, intersections)
                        #returns the best cell
                        cv2.circle(img, vanishing_point[0], 5, (10, 10, 10), thickness=2)

                        cv2.imshow('vp Image', img)

            except Exception as e:
                print ("Error - " + str(e))

        cv2.destroyAllWindows()


    def vpFromImg(self):
        filepath = "cor_in.jpg"

        img = cv2.imread(filepath)
        img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)

        #try:
        hough_lines = hough_transform(img, False) #
        if hough_lines:
            random_sample = getLineSample(hough_lines, 100) #take a sample of n lines
            intersections = find_intersections(random_sample, img) #Find intersections in the sample
            if intersections: #if intersections are found
                grid_rows = 2
                grid_columns = 5

                grid_size[0] = img.shape[0] //25
                grid_size[1] = img.shape[1] //25
                vanishing_point = vp_candidates(img, grid_size, intersections)
                print str(vanishing_point)
                #cv2.rectangle(img, (100, 100), (150, 150), (0, 255, 0), 2)
                cv2.circle(img, vanishing_point[0], 5, (10,10,10),thickness=2)
                cv2.imshow('vp Image',img)

        # img2 = cv2.imread(filepath)
        # img2 = cv2.resize(img2, (0, 0), fx=0.2, fy=0.2)
        #
        # gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        # cv2.imshow("edges",edges)
        # minLineLength = 10
        # maxLineGap = 20
        # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength, maxLineGap)
        # for line in lines:
        #     for x1, y1, x2, y2 in line:
        #         cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #
        # for line in lines:
        #     for x1, y1, x2, y2 in line:
        #         grad = (y2-y1)/(x2-x1)
        #         if grad == 0 or np.isnan(grad):
        #             continue
        #         r = 100
        #         dy = np.sqrt(((r**2)*(grad**2))/(1+grad**2))
        #         dx = dy/grad
        #
        #         x1p = int(x1+dx)
        #         y1p = int(y1+dy)
        #         x2p = int(x2-dx)
        #         y2p = int(y2-dy)
        #
        #         # Draw a red line
        #         cv2.line(img2, (x1p, y1p), (x2p, y2p), (0, 0, 255), 2)
        #
        # cv2.imshow("houghP", img2)

        # except Exception as e:
        #     print ("Error - " + str(e))

        cv2.waitKey(0) != 27
        cv2.destroyAllWindows()



if __name__ == '__main__':
    cam = True

    vpThread = CameraThreadClass(1,"vpThread")
    vpThread.start()
    threadQueue.append(vpThread)

    ##do what you want joe

    uInput = ""
    while uInput != "q":
        uInput = input("press q to exit...")


    #between these two points


    for t in threadQueue:
        t.join()

    print "stopping..."
##end