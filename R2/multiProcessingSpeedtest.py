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
    #def __init__(self, name):
    #     self.name = name
    #     #self.count = args[0]
    #    self._target = self.run()
    #     #self._args = tuple(args)
    #     self._name = name or type(self).__name__ + '-' + \
    #                          ':'.join(str(i) for i in self._identity)

    def run(self):
        #print self._args
        print self._args[1][0]
        print "printing 1 to count"
        for i in range(0, self._args[0]):
            print str(i)

        print self._args[1][1]
        self._args[1][1] = "im fine"

    def getVP(self):
        print "counting from 1 to 50"
        for i in range(0, 50):
            print str(i)

def writeThings (num, dataList):
    print dataList[0]
    print "printing 1 to count"
    for i in range(0, num):
        print str(i)

    print dataList[1]
    dataList.append("im fine")
    print dataList[2]

if __name__ == '__main__':
    mgr = multiprocessing.Manager()
    l = mgr.list(["hi", "how are you?"])
    l[0] = "hola"
    #newProcess = cameraProcess(name="camProc", args=(50,l,))
    #newProcess.start()
    newProc = multiprocessing.Process(target=writeThings, args=(50,l,))
    newProc.start()

    time.sleep(2)
    newProc.join()
    print l