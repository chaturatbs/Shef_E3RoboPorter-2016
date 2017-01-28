
import cv2
import numpy as np
import os
#import MSAC
capWebcam = cv2.VideoCapture(1)         # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam


if capWebcam.isOpened() == False:               # check if VideoCapture object was associated to webcam successfully
    print "error: capWebcam not accessed successfully\n\n"      # if not, print error message to std out
    os.system("pause")                                          # pause until user presses a key so user can see error message                                                      # and exit function (which exits program)

while cv2.waitKey(1) != 27 and capWebcam.isOpened():
    blnFrameReadSuccessfully, imgOriginal = capWebcam.read()

    if not blnFrameReadSuccessfully or imgOriginal is None:  # if frame was not read successfully
        print "error: frame not read from webcam\n"  # print error message to std out
        os.system("pause")  # pause until user presses a key so user can see error message
        break

    #img = cv2.imread(imgOriginal)
    #img = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
    gray = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
    #blurred = cv2.blur(gray, (9,9))
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 80, 150, apertureSize=3)
    #cv2.imshow("edges", edges)
    lines = cv2.HoughLines(edges,1,np.pi/180,50)

    try:
        for i in range(0, 50):
            for rho, theta in lines[i]:
                #if theta < 5*np.pi/180
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                cv2.line(gray, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # cv2.imwrite('houghlines3.jpg',img)
        #cv2.imshow('transformed', gray)
    except:
        print ("error")

    #cv2.namedWindow("imgOriginal", cv2.WINDOW_NORMAL)  # create windows, use WINDOW_AUTOSIZE for a fixed window size
    cv2.namedWindow("edges", cv2.WINDOW_NORMAL)
    cv2.namedWindow("transformed", cv2.WINDOW_NORMAL)

    #cv2.imshow("imgOriginal", imgOriginal)  # show windows
    cv2.imshow("edges", edges)
    cv2.imshow("transformed", gray)

    #estVP = MSAC.init(IMG_WIDTH, IMG_HEIGHT, 1);
    #msac.singleVPEstimation(lines, & number_of_inliers, vanishing_point);

cv2.waitKey(0) != 27
cv2.destroyallwndows

