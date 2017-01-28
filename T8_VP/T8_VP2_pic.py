
import cv2
import numpy as np
#import MSAC

img = cv2.imread('cor_in.jpg')
img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 250, 350, apertureSize = 5)

cv2.imshow("edges", edges)
lines = cv2.HoughLines(edges,2,np.pi/180,350)

for i in range(0, len(lines)):
    for rho,theta in lines[i]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))

        cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imwrite('houghlines3.jpg',img)

cv2.imshow('transformed wCanny', img)

clusters = np.zeros(1)
nInliers = np.zeros(1)

#vps = np.zeros(1)
#vpEstimator = MSAC.MSAC(1,[len(img),len(np.transpose(img))],True)
#vpOutput = vpEstimator.multipleVPEstimation(lines, clusters, nInliers, vps, 1)

#print vpOutput

cv2.waitKey(0) != 27
cv2.destroyAllWindows()

