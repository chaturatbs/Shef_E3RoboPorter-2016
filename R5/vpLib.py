
#Based on code by Github user SZanlongo

import random
import cv2
import numpy as np
import time
import operator


# Perform edge detection
def hough_transform(img, probabilistic, startTime, profiling, verbose):
    sumTime = 0
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # Convert image to grayscale

    if profiling:
        duration = time.time() - startTime
        if verbose:
            print "Gray Time :" + str(duration)
        sumTime += duration
        startTime = time.time()

    #define a kernel for morphological noise reduction
    #compare this with a blur
    kernel = np.ones((7, 7), np.uint8)
    opening = cv2.morphologyEx(gray, cv2.MORPH_OPEN, kernel)  #erode, then dilate

    if profiling:
        duration = time.time() - startTime
        if verbose:
            print "Morph Time :" + str(duration)
        sumTime += duration
        startTime = time.time()

    #detect Edges
    edgeThreshhold  = 60
    edges = cv2.Canny(opening, 30, 80, apertureSize=3)  # Canny edge detection

    if profiling:
        duration = time.time() - startTime
        if verbose:
            print "Canny Time :" + str(duration)
        sumTime += duration
        startTime = time.time()

    #Standard Hough transform
    lines = cv2.HoughLines(edges, 1, np.pi / 45, 50)
    hough_lines = []

    if profiling:
        duration = time.time() - startTime
        if verbose:
            print "Lines Time :" + str(duration)
        sumTime += duration
        startTime = time.time()

    # Convert Rho and Theta to a line with end points
    thetaThreshold = 10 #to filter out lines with shallow angles
    for line in range(0,min(len(lines),20)):#lines:
        for rho, theta in lines[line]:

            #Filter out vertical and horizontal lines
            if (theta < thetaThreshold*np.pi/180) or(theta > (180-thetaThreshold)*np.pi/180) or (abs(theta) > (90-thetaThreshold)*np.pi/180 and abs(theta) < (90+thetaThreshold)*np.pi/180):
                continue
            else:
                a = np.cos(theta)
                b = np.sin(theta)

                #set start point
                x0 = a * rho
                y0 = b * rho

                #extend line +- 1000 from start point
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))


                #Draw a red line
                #cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 1)
                #add the line to observed line list
                hough_lines.append(((x1, y1), (x2, y2)))

    if profiling:
        duration = time.time() - startTime
        if verbose:
            print "Cart Time :" + str(duration)
        sumTime += duration
        startTime = time.time()

    return hough_lines, startTime, sumTime #return the lines found

# Random sampling of lines
def getLineSample(lines, size):
    if size > len(lines): #sanity check
        size = len(lines)
    return random.sample(lines, size) #return a random sample of lines

# Determinant
def det(a, b):
    return a[0] * b[1] - a[1] * b[0]

# Find intersection point of two lines (not segments!)
def line_intersection(line1, line2):
        #line[0] is start point, line[1] is endpoint
        #line[][0] is x, line [][1] is y

    x_diff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    y_diff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    grad1 = y_diff[0] / x_diff[0]
    grad2 = y_diff[1] / x_diff[1]

    if ((grad1 >= 0) and (grad2 >= 0)) or ((grad1 < 0) and (grad2 < 0)):
        return None

    #Need to check this
    #find the determinant
    div = det(x_diff, y_diff)
    if div == 0:
        return None  # Lines don't cross

    #if intersects..
    d = (det(*line1), det(*line2))
    x = det(d, x_diff) / div
    y = det(d, y_diff) / div
    #------

    #if intersects, return intersection points
    return x, y

# Find multi line intersections
def find_intersections(lines, img):
    intersections = []
    for i in xrange(len(lines)): #for all lines
        line1 = lines[i] #for each line
        for j in xrange(i + 1, len(lines)): #for every other line
            line2 = lines[j] #for each, other line

            #for un-equal line pairs, check intersection
            if not line1 == line2: #given lines aren't the same
                intersection = line_intersection(line1, line2) #check for line intersection
                if intersection:  # If lines cross...
                    # Don't include intersections that happen off-image
                    # Seems to cost more time than it saves
                    # if not (intersection[0] < 0 or intersection[0] > img.shape[1] or
                    #                 intersection[1] < 0 or intersection[1] > img.shape[0]):
                    # print 'adding', intersection[0],intersection[1],img.shape[1],img.shape[0]
                    intersections.append(intersection) #...add to the line intersections

    return intersections

# Voting Process to find the grid with most intersection => vanishing point
def find_vanishing_point(img, grid_size, intersections):
    # Image dimensions
    image_height = img.shape[0]
    image_width = img.shape[1]

    # Grid dimensions
    grid_rows = (image_height // grid_size[0]) +1
    grid_columns = (image_width // grid_size[1]) + 1

    # Current cell with most intersection points
    max_intersections = 0
    best_cell = None


    for i in xrange(grid_rows):
        for j in xrange(grid_columns):
            #define and draw grids
            cell_left = j * grid_size[1]
            cell_right = (j + 1) * grid_size[1]
            cell_top = i * grid_size[0]
            cell_bottom = (i + 1) * grid_size[0]
            cv2.rectangle(img, (cell_left, cell_top), (cell_right, cell_bottom), (0, 0, 0), 1)

            current_intersections = 0  # Number of intersections in the current cell
            #check if there are intersections in the current grid
            #i = 0
            for x, y in intersections:
                #i += 1
                if cell_left < x < cell_right and cell_top < y < cell_bottom:
                    #increase the count if intersection is in the grid
                    current_intersections += 1
                    #del intersections[i]
                    #maybe remove the observed intersection from the list to speed up??

            # If Current cell has more intersections the best observed...
            if current_intersections > max_intersections:
                max_intersections = current_intersections
                best_cell = ((cell_left + cell_right) / 2, (cell_bottom + cell_top) / 2)

    #cv2.imshow("mid Grid", img)
    if not best_cell == [None, None]:
        rx1 = best_cell[0] - grid_size[1] / 2
        ry1 = best_cell[1] - grid_size[0] / 2
        rx2 = best_cell[0] + grid_size[1] / 2
        ry2 = best_cell[1] + grid_size[0] / 2
        cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (0, 255, 0), 2)

    return best_cell

def vp_candidates(img, grid_size, intersections):

    intersectionCounts = {}
    bestIntersections = {}
    # Image dimensions
    image_height = img.shape[0]
    image_width = img.shape[1]

    #print intersections

    # Grid dimensions
    grid_rows = (image_height // grid_size[0]) + 1
    grid_columns = (image_width // grid_size[1]) + 1

    # Current cell with most intersection points
    max_intersections = 0
    best_cell = None

    for i in intersections:
        cell_left = i[0] - (grid_size[1] / 2)
        cell_right = i[0] + (grid_size[1] / 2)
        cell_top = i[1] - (grid_size[0] / 2)
        cell_bottom = i[1] + (grid_size[0] / 2)

        #print i, cell_left, cell_right, cell_top, cell_bottom

        #cv2.rectangle(img, (cell_left, cell_top), (cell_right, cell_bottom), (0, 0, 0), 1)

        current_intersections = 0  # Number of intersections in the current cell
        # check if there are intersections in the current grid
        # i = 0
        for x, y in intersections:
            # i += 1
            if cell_left < x < cell_right and cell_top < y < cell_bottom:
                # increase the count if intersection is in the grid
                current_intersections += 1
                # del intersections[i]
                # maybe remove the observed intersection from the list to speed up??
        if current_intersections != 0:  # if non zero intersections...
            intersectionCounts[i] = current_intersections  # ...keep trac
        # If Current cell has more intersections the best observed...
        if current_intersections > max_intersections:
            max_intersections = current_intersections
            best_cell = ((cell_left + cell_right) / 2, (cell_bottom + cell_top) / 2)

    #draw a rectangle around the best candidate
    if not best_cell == [None, None]:
        rx1 = best_cell[0] - grid_size[1] / 2
        ry1 = best_cell[1] - grid_size[0] / 2
        rx2 = best_cell[0] + grid_size[1] / 2
        ry2 = best_cell[1] + grid_size[0] / 2
        #cv2.rectangle(img, (rx1, ry1), (rx2, ry2), (0, 255, 0), 2)

    #print intersectionCounts

    testKey = ""
    #find the top 5 candidates for the vanishing point
    #for i in range(0,5):
    #    testKey = str(max(intersectionCounts, key=lambda key: intersectionCounts[key]))
    #   bestIntersections[testKey] = intersectionCounts[testKey]
    #    del intersectionCounts[testKey]

    #print bestIntersections

    #print (sorted(bestIntersections.items(), key=lambda x: x[1]))

    return (best_cell,bestIntersections)
