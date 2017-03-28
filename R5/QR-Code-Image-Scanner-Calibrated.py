#!/usr/bin/python
from sys import argv
import zbar
import time
from PIL import Image
from msvcrt import getch

# create a Scanner
scanner = zbar.ImageScanner()

# configure the Scanner
scanner.parse_config('enable')


while(True):
    # Obtain Image Data
    pil = Image.open('cam.jpg').convert('L')
    width, height = pil.size
    raw = pil.tobytes()

    # wrap image data
    image = zbar.Image(width, height, 'Y800', raw)
    print "Ready..."

    # scan the image for QR Codes
    scanner.scan(image)

    # When symbol detected
    for symbol in image:
        # Seperate code into useful data
        data = symbol.data.split(',')
        location = data[0]
        size = float(data[1])
        # Save code corners
        x0 = symbol.location[0][0]
        x1 = symbol.location[1][0]
        x2 = symbol.location[2][0]
        x3 = symbol.location[3][0]
        y0 = symbol.location[0][1]
        y1 = symbol.location[1][1]
        y2 = symbol.location[2][1]
        y3 = symbol.location[3][1]

        # Calculate x and y centre points
        x_centre = (x0 + x1 + x2 + x3) / 4
        y_centre = (y0 + y1 + y2 + y3) / 4

        # Calculate average pixel width
        P_x = (abs(x_centre - x0) + abs(x_centre - x1) + abs(x_centre - x2) + abs(x_centre - x3)) / 2
        P_y = (abs(y_centre - y0) + abs(y_centre - y1) + abs(y_centre - y2) + abs(y_centre - y3)) / 2
        P = (P_x + P_y) / 2

        # Set Code actual size in mm
        F = 673
        W = size
        D = ((F * W) / P) / 10
        # print 'F = ', F,'W = ', W, 'P = ', P, 'D = ', ((F * W) / P)
        print 'QR Code scanned:', symbol.data
        print 'RoboPorter is', "%.2fcm" %D, 'away from ' '%s' % data[0]
        time.sleep(1)