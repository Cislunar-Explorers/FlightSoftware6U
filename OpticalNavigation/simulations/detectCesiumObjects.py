import numpy as np
#import scipy as sp
import cv2
import argparse
import copy
import math

from core.find import round_up_to_odd

TEMPLATE_SUN_PATH = "C:\\Users\\easha\\Downloads\\UnityTemplateSun.PNG"

"""
Replace STK/Cesium Sun with custom image.
[583. 421.   6.]
"""
CesiumSun_HSV_BOUNDARIES = [([10, 100, 20], [40, 255, 50])]
CesiumRedMoon_HSV_BOUNDARIES = [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [179, 255, 255])]

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

"""
Use this function for detecting the yellow Cesium Sun and replacing it with a new Sun image.
Precondition: Cesium Sun is generated using the following settings
in GenerateCesium.html:
scene.sun.glowFactor = 1;
scene.sun.sunBloom = true;
[image]: cv2 image where we are looking for the Sun
[path]: new Sun image to replace the current Sun image with
Returns:
image with new Sun, sun X and Y pos, sun radius (of new Sun)
"""
def detectAndReplaceCesiumSun(image, path):
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
        return image[padSize:-padSize, padSize:-padSize,:], x - padSize, y - padSize,  6
    return None

# TODO: Add Sun and Earth Detection

"""
Use this function for detecting the Cesium Moon with red texture.
Precondition: Cesium Moon is generated using the following settings
in GenerateCesium.html:
var isSimulatingForGroundTruthMeasurements = true;
[image]: cv2 image where we are looking for the red Moon
"""
def detectCesiumRedMoon(image):
    boundaries = CesiumRedMoon_HSV_BOUNDARIES
    # Replacement sun
    upsampledImage, upsampleFactor = cv2.pyrUp(cv2.pyrUp(cv2.pyrUp(image))), 2*2*2

    original_hsv = cv2.cvtColor(upsampledImage, cv2.COLOR_BGR2HSV)

    output = None
    mask = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        if mask is None:
            mask = cv2.inRange(original_hsv, lower, upper)
        else:
            mask += cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)
        
    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)

    # Increase gray mask brightness for non-black pixels only
    gray[gray > 1] = 255

    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(11))

    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=30, minRadius=1, maxRadius=0)
    if circles is not None:
        circle = circles[0][0]
        print(circle)
        x = int((circle[0]+5)/upsampleFactor) # The detected circle is always a bit off-center
        y = int((circle[1]+5)/upsampleFactor)
        r = int(circle[2]/upsampleFactor)
        r = int(r * 1.5) # The detected circle is always a bit smaller than is should be
        x1 = int(circle[0])
        y1 = int(circle[1])
        cv2.circle(upsampledImage, (x*upsampleFactor, y*upsampleFactor), r*upsampleFactor, (255, 255, 255), 1)
        cv2.circle(upsampledImage, (x*upsampleFactor, y*upsampleFactor), 2, (0, 255, 255), 1)
        return upsampledImage[y1-100:y1+100, x1-100:x1+100], x, y, r
    return None, None, None, None

if __name__ == "__main__":
    """
    Run "python FilterCesiumSun.py -i=<IMAGE>" to test this module
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help = "path to the base image")
    args = vars(ap.parse_args())
    
    true_image = cv2.imread(args["image"])
    # img = detectCesiumSun(true_image, TEMPLATE_SUN_PATH)  
    img, _, _, _ = detectCesiumRedMoon(true_image)      
    if img is not None:
        cv2.imshow("Result", img)
        cv2.waitKey(0)
    else:
        print("NO circle")

