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
    def __init__(self, name, args):
        self.name = name
        self._target = self.run()
        self._args = tuple(args)
        self._name = name or type(self).__name__ + '-' + \
                             ':'.join(str(i) for i in self._identity)

    def run(self):
        print self._args
        print "printing 1 to count"
        # for i in range(0, self.count):
        #     print str(i)

    def getVP(self):
        print "counting from 1 to 50"
        for i in range(0, 50):
            print str(i)


if __name__ == '__main__':
    newProcess = cameraProcess("camProc", (20))
    #newProcess.start()