import os
import numpy as np

from vpLib import *
grid_size = [0,0]


def vpFromCam():
    capWebcam = cv2.VideoCapture(0)  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam
    if capWebcam.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
        print "error: capWebcam not accessed successfully\n\n"  # if not, print error message to std out
        os.system("pause")

    while cv2.waitKey(1) != 27 and capWebcam.isOpened():
        blnFrameReadSuccessfully, img = capWebcam.read()

        if (not blnFrameReadSuccessfully) or (img is None):  # if frame was not read successfully
            print "error: frame not read from webcam\n"  # print error message to std out
            os.system("pause")  # pause until user presses a key so user can see error message
            break

        try:
            hough_lines = hough_transform(img, False)  #
            if hough_lines:
                random_sample = getLineSample(hough_lines, 100)  # take a sample of n lines
                intersections = find_intersections(random_sample, img)  # Find intersections in the sample
                if intersections:  # if intersections are found
                    grid_size[0] = img.shape[0] // 20
                    grid_size[1] = img.shape[1] // 20
                    vanishing_point = find_vanishing_point(img, grid_size, intersections)

                    cv2.imshow('vp Image', img)

        except Exception as e:
            print ("Error - " + str(e))

    cv2.destroyAllWindows()


def vpFromImg():
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

            grid_size[0] = img.shape[0] //30
            grid_size[1] = img.shape[1] //30
            vanishing_point = find_vanishing_point(img, grid_size, intersections)

            cv2.imshow('vp Image',img)

    img2 = cv2.imread(filepath)
    img2 = cv2.resize(img2, (0, 0), fx=0.2, fy=0.2)

    gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    cv2.imshow("edges",edges)
    minLineLength = 10
    maxLineGap = 20
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength, maxLineGap)
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(img2, (x1, y1), (x2, y2), (0, 255, 0), 2)

    for line in lines:
        for x1, y1, x2, y2 in line:
            grad = (y2-y1)/(x2-x1)
            if grad == 0 or np.isnan(grad):
                continue
            r = 100
            dy = np.sqrt(((r**2)*(grad**2))/(1+grad**2))
            dx = dy/grad

            x1p = int(x1+dx)
            y1p = int(y1+dy)
            x2p = int(x2-dx)
            y2p = int(y2-dy)

            # Draw a red line
            cv2.line(img2, (x1p, y1p), (x2p, y2p), (0, 0, 255), 2)

    cv2.imshow("houghP", img2)

    # except Exception as e:
    #     print ("Error - " + str(e))

    cv2.waitKey(0) != 27
    cv2.destroyAllWindows()

cam = True

if cam:
    vpFromCam()
else:
    vpFromImg()
