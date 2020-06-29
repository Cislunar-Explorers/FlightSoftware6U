import numpy as np
#import scipy as sp
import cv2
import argparse
import copy
import math

TEMPLATE_SUN_PATH = "C:\\Users\\easha\\Downloads\\UnityTemplateSun.PNG"

"""
Replace STK/Cesium Sun with custom image.
[583. 421.   6.]
"""
CesiumSun_HSV_BOUNDARIES = [([10, 100, 20], [40, 255, 50])]

"""
Use this function for cropping the template sun image
to the desired size.
Returns a centered cropped sun.
"""
def fitCircleAroundReplacement():
    x = 583
    y = 421
    truer = 6
    r = truer * 60
    path = "C:\\Users\\easha\\Downloads\\UnityTemplateSun.PNG"
    image = cv2.imread(path)
    # cv2.circle(image, (x, y), r, (255, 255, 255), 2)
    # cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
    # cv2.imshow('fit', image[y-r:y+r,x-r:x+r])
    # uncomment to crop sun image
    # cv2.imwrite(path, image[y-r:y+r,x-r:x+r])
    # cv2.waitKey(0)
    return image[y-r:y+r,x-r:x+r]

def detectCesiumSun(image, path):

    boundaries = CesiumSun_HSV_BOUNDARIES

    # Replacement sun
    newSun = cv2.imread(path)
    assert(newSun.shape[0] == newSun.shape[1])
    padSize = newSun.shape[0]
    image = np.pad(image, [(padSize, padSize), (padSize, padSize), (0,0)], mode='constant', constant_values=0)

    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    # Increase gray mask brightness for non-black pixels only
    gray[gray > 1] = 255

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=15, minRadius=1, maxRadius=0)
    if circles is not None:
        circle = circles[0][0]
        print(circle)
        if circle is not None:
                x = int(circle[0])
                y = int(circle[1])
                r = int(circle[2])
                # Clear out current sun pixels
                image[y-y:y+r, x-r:x+r] = 0
                # Plot new sun
                newRad = int(padSize/2)
                print(y-newRad,y+newRad, x-newRad,x+newRad)
                image[y-newRad:y+newRad, x-newRad:x+newRad] = newSun
                # cv2.circle(image, (x, y), r, (255, 255, 255), 2)
                # cv2.circle(image, (x, y), 2, (0, 0, 255), 1)

        return image[padSize:-padSize, padSize:-padSize,:]
    return None


if __name__ == "__main__":
    """
    Run "python FilterCesiumSun.py -i=<IMAGE>" to test this module
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help = "path to the base image")
    args = vars(ap.parse_args())
    
    true_image = cv2.imread(args["image"])
    img = detectCesiumSun(true_image, TEMPLATE_SUN_PATH)    
    if img is not None:
        cv2.imshow("Result", img)
        cv2.waitKey(0)

