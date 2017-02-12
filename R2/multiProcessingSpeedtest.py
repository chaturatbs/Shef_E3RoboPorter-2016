#from python docs

#
# Simple benchmarks for the multiprocessing package
#
# Copyright (c) 2006-2008, R Oudkerk
# All rights reserved.
#

import time, sys, multiprocessing, threading, Queue, gc

if sys.platform == 'win32':
    _timer = time.clock
else:
    _timer = time.time

delta = 1


#### TEST_QUEUESPEED
class cameraProcess(multiprocessing.Process):
    def __init__(self):
        pass

    def run(self):
        target = self.getVP()

    def getVP(self):
        print "counting from 1 to 50"
        for i in range(0, 50):
            print str(i)


if __name__ == '__main__':
    newProcess = cameraProcess()
    newProcess.run()