
import cv2
import numpy
import os
import multiprocessing
import time
import vpLib


grid_size = [0,0]
global vanish
global vpValid

vpValid = multiprocessing.Value('b', False)
vanish = multiprocessing.Value("d",0)


def medianFilter(vpCoord, n, vanishx, vanishy):
    i = 0
    #global vpValid

    vanish = [0.,0.]
    #print len(vpCoord)
    if len(vpCoord) == 2:
        #print vpCoord
        #print vanishx, vanishy
        vanishx[0] = vanishx[1]
        vanishx[1] = vanishx[2]
        vanishx[2] = vpCoord[0]

        vanishy[0] = vanishy[1]
        vanishy[1] = vanishy[2]
        vanishy[2] = vpCoord[1]

        sortedx = sorted(vanishx)
        sortedy = sorted(vanishy)

        medVanish = (sortedx[1],sortedy[1])


    return medVanish, vanishx, vanishy

def varianceFilter(vpCoord, n, varx, vary):
    global vpValid
    i = 0

    while (len(varx) < n):
        varx.append(0)

    while (len(vary) < n):
        vary.append(0)

    for i in range(0, n - 1):
        varx[i] = varx[i + 1]
        vary[i] = vary[i + 1]

    varx[n - 1] = vpCoord[0]
    vary[n - 1] = vpCoord[1]

    medVar = numpy.var(varx[0:n-1]), numpy.var(vary[0:n-1])

    if (medVar[0] > 1000) or (medVar[1] > 150):
        vpValid.Value = 0
    else:
        vpValid.Value = 1

    return varx, vary

def vpFromCam():
    global vanish
    global vpValid

    profiling = False
    vanishx = [0,0,0]
    vanishy = [0,0,0]
    varx = [0, 0, 0, 0, 0]
    vary = [0, 0, 0, 0, 0]
    capVid = cv2.VideoCapture(0)

    #capVid = cv2.VideoCapture('testOutsideLab.avi')  # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    outvid = cv2.VideoWriter('output.avi', fourcc, 5.0, (640,480))

    while not capVid.isOpened():
        pass

    centre_point = capVid.read()[1].shape[1] / 2

    if capVid.isOpened() == False:  # check if VideoCapture object was associated to webcam successfully
        print "error: capVid not accessed successfully\n\n"  # if not, print error message
        #logging.error("error: capWebcam not accessed successfully\n\n")
        os.system("pause")

    while cv2.waitKey(1) != 27 and capVid.isOpened():
        blnFrameReadSuccessfully, img = capVid.read()
        #origimg = img
        startTime = time.time()
        sumTime = 0
        #outvid.write(img)

        #print "Image Loaded: " + str(startTime)

        # if len(frameNumber) > 0:
        #     frameNumber.append(frameNumber[len(frameNumber) - 1] + 1)
        # else:
        #     frameNumber[0] = 1

        try: #try to find vanishing point
            hough_lines, startTime, sumTime = vpLib.hough_transform(img, False, startTime, profiling)  #calculate hough lines
            #print str(hough_lines)
            if hough_lines: #if lines found
                random_sample = vpLib.getLineSample(hough_lines, 30)  # take a sample of 100 line
                intersections = vpLib.find_intersections(random_sample, img)  # Find intersections in the sample

                if profiling:
                    duration = time.time() - startTime
                    print "Intersection Time :" + str(duration)
                    sumTime += duration
                    startTime = time.time()

                #print str(intersections)
                if intersections:  # if intersections are found
                    grid_size[0] = img.shape[0] // 8 #set the grid size to be 20 by 20
                    grid_size[1] = img.shape[1] // 20
                    #find vanishing points
                    vanishing_point = vpLib.vp_candidates(img, grid_size, intersections)
                    #returns the best cell
                    #print vanishing_point
                    vanish2, vanishx, vanishy = medianFilter(vanishing_point[0], 3, vanishx, vanishy)
                    varx, vary = varianceFilter(vanishing_point[0], 5, varx, vary)

                    if vpValid.Value == 1:
                        cv2.circle(img, (vanish2[0], vanish2[1]), 5, (210, 255, 10), thickness=2)
                    else:
                        cv2.circle(img, (vanish2[0], vanish2[1]), 5, (10, 10, 255), thickness=2)

                    vanish.Value = int(vanish2[0]-centre_point)
                else:
                    vpValid.Value = 0
            else:
                vpValid.Value = 0
            cv2.imshow('vp Image', img)

            #time.sleep(0.25)

            if profiling:
                duration = time.time() - startTime
                print "Finish Time :" + str(duration)
                sumTime += duration
                startTime = time.time()

            expectedFPS = 1/sumTime

            print "Expected FPS: " + str(expectedFPS)

            #fps.append(expectedFPS)
            # plt.plot(frameNumber, fps)
            # plt.pause(0.05)

            print "----------------------------------------------"

        except Exception as e:
            pass

    capVid.release()
    outvid.release()

    cv2.destroyAllWindows()


if __name__ == '__main__':

    multiprocessing.freeze_support()
    mpManager = multiprocessing.Manager()

    vpFromCam()
