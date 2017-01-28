
class multiThreadBase(threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.avgRuntime = 0.
        self.startTime = 0.
        self.endTime = 0.
        self.avgCounter = 0
        self.loopDebugInterval = 10
        self.debug = False

    def loopStartFlag(self):
        self.startTime = time.localtime()
        pass

    def loopEndFlag(self):
        self.startTime = time.localtime()
        pass

    def loopRunTime(self):
        self.avgRuntime += ((self.endTime - self.startTime) - self.avgRuntime) / self.loopDebugInterval
        self.avgCounter += 1

        if self.avgCounter == self.loopDebugInterval:
            print (self.name + " Avg loop Runtime - " + str(self.avgRuntime))
            self.avgCounter = 0
