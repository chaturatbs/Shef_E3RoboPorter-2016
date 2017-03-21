import numpy
import copy
import time
import cv2
#import plotly as plt
#import plotly.graph_objs as go

import matplotlib as mpl
from matplotlib import pyplot

recPenalty = 1
momentumBonus = 0
tempX = 0
tempY = 0

class agent:

    locX = 0
    locY = 0
    dist = 0
    destX = 0
    destY = 0
    envScore = []
    distScore = []
    score = []
    pathmemory = []
    alpha = 10
    beta = 100

    def __init__(self):
        locX = 0
        locY = 0
        dist = 0
        destX = 0
        destY = 0

    def moveFwd(self):
        self.locX -= 1

    def moveBack(self):
        self.locX += 1

    def moveLeft(self):
        self.locY -= 1

    def moveRight(self):
        self.locY += 1

    def finddist(self):
        self.dist = numpy.sqrt(numpy.square(self.locX - self.destX) + numpy.square(self.locY - self.destY))
        return self.dist

    def setpos(self, x, y):
        self.locY = y
        self.locX = x

    def setdest(self, x, y):
        self.destY = y
        self.destX = x

    def move(self, to, rev):

        if to == 0:
            if rev:
                self.moveBack()
            else:
                self.moveFwd()
        elif to == 1:
            if rev:
                self.moveFwd()
            else:
                self.moveBack()
        elif to == 2:
            if rev:
                self.moveLeft()
            else:
                self.moveRight()
        elif to == 3:
            if rev:
                self.moveRight()
            else:
                self.moveLeft()

    def checkquad(self, envmap):
        self.checkdist()
        self.envCheck(envmap)
        self.calcHeuristic()

    def checkdist(self):
        self.distScore = []
        for motion in range(0, 4):
            self.move(motion, 0)
            self.distScore.append(self.finddist())
            self.move(motion, 1)
        print("dist score is " + str(self.distScore))
        print("min dist score is " + str(min(self.distScore)) + " and its at " +
              str(self.distScore.index(min(self.distScore))))
        return self.distScore.index(min(self.distScore))

    def envCheck(self, envmap):
        self.envScore = []
        for motion in range(0, 4):
            self.move(motion, 0)
            if 0 <= self.locX < len(envmap) and 0 <= self.locY < len(envmap[1]):
                self.envScore.append(envmap[self.locX][self.locY])
            else:
                self.envScore.append(-10000)
            self.move(motion, 1)
        print("environment score is " + str(self.envScore))
        print("max env score is " + str(max(self.envScore)) + " and its at " +
              str(self.envScore.index(max(self.envScore))))
        return self.envScore.index(min(self.envScore))

    def calcHeuristic(self):
        self.score = []
        for i in range(0, 4):
            if self.distScore[i] != 0:
                self.score.append(self.alpha*self.envScore[i] + self.beta/self.distScore[i])
            else:
                self.score.append(self.alpha * self.envScore[i]+1)
            if self.envScore[i] == -10000:
                self.score[i] = -10000

        if len(self.pathmemory) != 0:
            print("momentum bonus in the ", self.pathmemory[len(self.pathmemory)-1], "direction")
            self.score[self.pathmemory[len(self.pathmemory)-1]] = self.score[self.pathmemory[len(self.pathmemory)-1]] + momentumBonus
        else:
            print('start of run, no momentum bonus')

        print("heuristic score is " + str(self.score))
        print("max heuristic score is " + str(max(self.score)) + " and its at " +
              str(self.score.index(max(self.score))))
        return self.score.index(max(self.score))

def onclick(event):
    global tempX
    global tempY
    print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          (event.button, event.x, event.y, event.xdata, event.ydata))
    tempX = event.xdata
    tempY = event.ydata
    pyplot.close()

def putmarker(motion):
    if motion == 0:
        return "^"
    elif motion == 1:
        return "b"
    elif motion == 2:
        return ">"
    elif motion == 3:
        return "<"

D = [[-10, 000, -10, 000, 000, 000, 000, -10, 000, 000],
     [-10, 000, -10, -10, -10, -10, -10, -10, 000, 000],
     [-10, 000, 000, 000, 000, 000, 000, -10, 000, 000],
     [-10, 000, -10, -10, -10, -10, 000, -10, 000, 000],
     [-10, 000, -10, 000, 000, -10, 000, -10, 000, 000],
     [-10, 000, -10, 000, 000, -10, 000, -10, 000, 000],
     [-10, 000, -10, 000, 000, -10, 000, -10, 000, 000],
     [-10, 000, -10, -10, -10, -10, 000, -10, 000, 000],
     [-10, 000, 000, 000, 000, 000, 000, 000, 000, 000],
     [-10, -10, -10, -10, -10, -10, -10, -10, -10, -10]]

C = [[000, 000, 000, 000, 000, 000, 000, 000, 000, 000],
     [000, -10, -10, -10, 000, -10, -10, -10, 000, 000],
     [000, -10, 000, 000, 000, 000, 000, -10, 000, 000],
     [000, -10, 000, -10, -10, -10, 000, -10, 000, 000],
     [000, -10, 000, -10, 000, -10, 000, -10, 000, 000],
     [000, -10, 000, -10, 000, -10, 000, -10, 000, 000],
     [0, -10, 000, 000, 000, 000, 000, -10, 000, 000],
     [0, -10, -10, -10, -10, -10, -10, -10, 000, 000],
     [000, 000, 000, 000, 000, 000, 000, 000, 000, 000],
     [ 0, -10, -10, -10, 00, -10, -10, -10, -10, -10]]
#
# A = [[-10, 000, -10, 000, 000, 000, 000, -10, 000, -10, 000, 000, -10, 000, -10, 000, 000, 000, -10, -10],
#      [-10, 000, -10, -10, -10, -10, -10, -10, 000, -10, -10, 000, -10, 000, -10, 000, 000, -10, -10, -10],
#      [-10, 000, 000, 000, 000, 000, 000, -10, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000],
#      [-10, 000, -10, -10, -10, -10, 000, -10, 000, -10, 000, 000, 000, -10, 000, 000, 000, 000, 000, -10],
#      [-10, 000, -10, 000, 000, -10, 000, -10, 000, -10, 000, -10, -10, -10, -10, -10, -10, 000, 000, 000],
#      [-10, 000, -10, 000, 000, -10, 000, -10, 000, 000, 000, -10, 000, -10, 000, 000, 000, 000, 000, -10],
#      [-10, 000, -10, 000, 000, -10, 000, -10, 000, -10, -10, -10, 000, -10, 000, 000, 000, 000, -10, -10],
#      [-10, 000, -10, 000, 000, -10, 000, -10, 000, -10, 000, 000, 000, -10, 000, 000, 000, 000, 000, -10],
#      [-10, 000, -10, -10, -10, -10, 000, -10, 000, -10, 000, -10, 000, -10, -10, -10, -10, 000, 000, 000],
#      [-10, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, 000, -10],
#      [-10, -10, -10, 000, -10, -10, -10, -10, -10, 000, -10, 000, 000, 000, 000, -10, 000, 000, -10, 000],
#      [-10, -10, -10, 000, -10, 000, 000, 000, -10, 000, 000, 000, 000, 000, 000, -10, 000, -10, -10, 000],
#      [-10, -10, -10, 000, -10, 000, 000, -10, -10, -10, 000, 000, -10, 000, 000, -10, 000, 000, -10, 000],
#      [-10, 000, 000, 000, -10, 000, -10, -10, 000, 000, 000, 000, -10, 000, 000, -10, -10, -10, -10, 000],
#      [-10, 000, 000, 000, 000, 000, 000, 000, 000, -10, 000, 000, -10, 000, 000, 000, 000, 000, 000, 000],
#      [-10, 000, 000, 000, -10, -10, -10, 000, -10, 000, 000, 000, -10, 000, 000, 000, -10, -10, 000, 000],
#      [-10, 000, 000, 000, -10, -10, -10, 000, -10, -10, 000, -10, -10, -10, 000, 000, 000, -10, 000, 000],
#      [-10, -10, -10, -10, -10, -10, 000, 000, -10, 000, 000, 000, -10, 000, 000, 000, 000, 000, 000, 000],
#      [-10, 000, 000, 000, 000, 000, 000, 000, -10, -10, 000, 000, 000, 000, -10, -10, 000, -10, 000, 000],
#      [-10, -10, -10, -10, -10, -10, -10, -10, -10, 000, 000, 000, 000, 000, -10, -10, 000, -10, 000, 000]]

#A = 10*A
#A = numpy.multiply(A, 10)

cat = agent()

#
# data = [
#     go.Surface(
#         z=A
#     )
# ]
#
# layout = go.Layout(
#     title='Mt Bruno Elevation',
#     autosize=False,
#     width=500,
#     height=500,
#     margin=dict(
#         l=65,
#         r=50,
#         b=65,
#         t=90
#     )
# )
# fig = go.Figure(data=data, layout=layout)
# #plt.offline.plot(fig, filename='elevations-3d-surface')
# plt.offline.plot([dict(z=A, type='surface'),
#     dict(z= path, showscale=False, type='line')])

#########
# make values from -5 to 5, for this example
#zvals = numpy.random.rand(100,100)*10-5

# make a color map of fixed colors

o = cv2.imread('map.bmp')
o = cv2.cvtColor(o,cv2.COLOR_BGR2GRAY)
A = numpy.zeros((o.shape[0], o.shape[1]))
for i in range(0,o.shape[0]):
    for j in range(0,o.shape[1]):
        if o[i,j] == 255:
            A[i,j] = 0
        else:
            A[i,j] = -10

Abase = copy.deepcopy(A)


fig = mpl.pyplot.figure()
#mpl.pyplot.ion()
cmap = mpl.colors.ListedColormap(['gray','magenta','cyan','blue', 'green', 'black', 'red'])
bounds=[-100,-9,-3*float(recPenalty), -2*float(recPenalty), -1*float(recPenalty), 0, 2, 6]
norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
pyplot.grid(True,color='white', which='both')

# tell imshow about color map so that only set colors are used
img = mpl.pyplot.imshow(A, interpolation='nearest', cmap = cmap,norm=norm)

# make a color bar
#mpl.pyplot.colorbar(img,cmap=cmap, norm=norm,boundaries=bounds,ticks=[-5,0,5])

cid = fig.canvas.mpl_connect('button_press_event', onclick)
mpl.pyplot.show()
####

cat.setdest(int(round(tempY)),int(round(tempX)))
A[cat.destX][cat.destY] = 1000

tempX=0
tempY=0

fig = mpl.pyplot.figure()
pyplot.grid(True,color='white', which='both')
img = mpl.pyplot.imshow(A, interpolation='nearest', cmap = cmap,norm=norm)
cid = fig.canvas.mpl_connect('button_press_event', onclick)

mpl.pyplot.show()

cat.setpos(int(round(tempY)),int(round(tempX)))

#motionPath = copy.deepcopy(A)
start = time.time()

# print("The current environment...")
# i = 0
# for row in A:
#     print(A[i])
#     i += 1

# go to X from S
# actions are LRFR
path = []
nodeList = [[], []]


cat.finddist()
#end = [0, 6]


print("Cat is at " + str(cat.locX) + "," + str(cat.locY) + " at a distance " + "{0:.2f}".format(cat.dist) + " to goal")
#motionPath[cat.locX][cat.locY] = "X"
nodeList[0].append(cat.locX)
nodeList[1].append(cat.locY)

# print("Motion Path... ")
# i = 0
# for row in motionPath:
#     print(motionPath[i])
#     i += 1

#cat.checkquad(A)
#
# img2 = mpl.pyplot.imshow(A, interpolation='nearest',cmap=cmap, norm=norm)
#
# mpl.pyplot.colorbar(img2, cmap=cmap, norm=norm, boundaries=bounds, ticks=[-5, 0, 5])
#
# mpl.pyplot.show()

while True:
    if cat.locX == cat.destX and cat.locY == cat.destY:
        finish = time.time()
        duration = finish - start
        print("it took " + str(duration) + " to finish the navigation")
        break

    cat.checkquad(A)
    path.append(cat.score.index(max(cat.score)))
    cat.pathmemory.append(cat.score.index(max(cat.score)))
    #motionPath[cat.locX][cat.locY] = putmarker(int(path[len(path)-1]))
    A[cat.locX][cat.locY] = A[cat.locX][cat.locY] - recPenalty

    #print("The current environment...")
    #i = 0
    #for row in A:
    #    print(A[i])
    #    i += 1

    cat.move(path[len(path)-1], 0)
    #motionPath[cat.locX][cat.locY] = "X"
    nodeList[0].append(cat.locX)
    nodeList[1].append(cat.locY)
    #print("Motion Path... ")
    #i = 0
    #for row in motionPath:
    #    print(motionPath[i])
    #   i += 1

    print("Cat is at " + str(cat.locX) + "," + str(cat.locY) + " at a distance " + "{0:.2f}".format(
        cat.dist) + " to goal")
    print("path is " + str(len(path)) + " steps long - " + str(path))
    print("__________________________________________")

    #mpl.pyplot.draw()

#Anew = numpy.subtract(A, Abase)
#Anew = -10*Anew

img2 = mpl.pyplot.imshow(A, interpolation='nearest',cmap=cmap, norm=norm)

mpl.pyplot.colorbar(img2, cmap=cmap, norm=norm, boundaries=bounds, ticks=[-5, 0, 5])

mpl.pyplot.show()

#fig = pyplot.figure(2)

#cmap2 = mpl.colors.LinearSegmentedColormap.from_list('my_colormap',['Gray', 'green','green','green', 'green', 'black', 'red'],256)
#img2 = mpl.pyplot.imshow(A, interpolation='nearest',cmap=cmap2, norm=norm)
#pyplot.colorbar(img2, cmap=cmap2)
#mpl.pyplot.show()

# print(start)
# s = A
# print(s)
# print(len(start))
#
# for i in start:
#     s = s[i]
#     print(s)