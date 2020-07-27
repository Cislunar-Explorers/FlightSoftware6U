import numpy as np
#import scipy as sp
import cv2
import argparse
import copy
import math
import os
from tqdm import tqdm

from core.find import round_up_to_odd
from core.preprocess import rectilinearToStereographicProjection

TEMPLATE_SUN_PATH = "C:\\Users\\easha\\Downloads\\UnityTemplateSun.PNG"

"""
Replace STK/Cesium Sun with custom image.
[583. 421.   6.]
"""
CesiumSun_HSV_BOUNDARIES = [([10, 100, 20], [40, 255, 50])]
CesiumRedMoon_HSV_BOUNDARIES = [([0, 100, 100], [10, 255, 255]), ([160, 100, 100], [179, 255, 255])]
CesiumEarth_HSV_BOUNDARIES = [([80,30,1], [160,254,254])]

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
def detectCesiumSun(image):
    boundaries = CesiumSun_HSV_BOUNDARIES
    # Replacement sun
    padSize = int(max(image.shape[0]/2, image.shape[1]/2))
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
        x = int(circle[0])
        y = int(circle[1])
        r = int(circle[2])
        # Clear out current sun pixels
        # image[y-y:y+r, x-r:x+r] = 0
        # Plot new sun
        # newRad = int(padSize/2)
        # print(y-newRad,y+newRad, x-newRad,x+newRad)
        # image[y-newRad:y+newRad, x-newRad:x+newRad] = newSun
        # cv2.circle(image, (x, y), r, (255, 255, 255), 2)
        # cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        return image[padSize:-padSize, padSize:-padSize,:], x - padSize, y - padSize,  r
    return None

"""
Use this function for detecting the Cesium Earth.
Precondition: Cesium Moon is generated using the following settings
in GenerateCesium.html:
var isSimulatingForGroundTruthMeasurements = true;
...
var globe = scene.globe;
globe.lightingFadeOutDistance = 0;
globe.lightingFadeInDistance = 0;
globe.nightFadeOutDistance = 0;
globe.nightFadeInDistance = 1;
[image]: cv2 image where we are looking for the Earth
Returns the rescaled coordinates and radius as well as an image of a circle placed over the body
"""
def detectCesiumEarth(image):
    boundaries = CesiumEarth_HSV_BOUNDARIES
    # Replacement sun
    upsampledImage, upsampleFactor = cv2.pyrUp(cv2.pyrUp(cv2.pyrUp(image))), 2*2*2

    padSize = int(max(upsampledImage.shape[0]/2, upsampledImage.shape[1]/2))
    upsampledImage = np.pad(upsampledImage, [(padSize, padSize), (padSize, padSize), (0,0)], mode='constant', constant_values=0)

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

    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(1))

    up_pad_circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=30, minRadius=1, maxRadius=0)
    if up_pad_circles is not None:
        up_pad_circle = up_pad_circles[0][0]
        x = int((up_pad_circle[0]-padSize)/upsampleFactor) # The detected circle is always a bit off-center
        y = int((up_pad_circle[1]-padSize)/upsampleFactor)
        r = int((up_pad_circle[2])/upsampleFactor)
        # r = int(r * 1.5) # The detected circle is always a bit smaller than is should be
        # x1 = int(up_pad_circle[0])
        # y1 = int(up_pad_circle[1])
        # r1 = int(up_pad_circle[2])
        cv2.circle(image, (x, y), r, (255, 0, 0), 2)
        cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        # uph, upw, _ = upsampledImage.shape
        return image, x, y, r
    return None

"""
Use this function for detecting the Cesium Moon with red texture.
Precondition: Cesium Moon is generated using the following settings
in GenerateCesium.html:
var isSimulatingForGroundTruthMeasurements = true;
[image]: cv2 image where we are looking for the red Moon
Returns the rescaled coordinates and radius as well as an image of a circle placed over the body
"""
def detectCesiumRedMoon(image):
    boundaries = CesiumRedMoon_HSV_BOUNDARIES
    # Replacement sun
    upsampledImage, upsampleFactor = cv2.pyrUp(cv2.pyrUp(cv2.pyrUp(image))), 2*2*2

    padSize = int(max(upsampledImage.shape[0]/2, upsampledImage.shape[1]/2))
    upsampledImage = np.pad(upsampledImage, [(padSize, padSize), (padSize, padSize), (0,0)], mode='constant', constant_values=0)

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


    gray_blurred = cv2.GaussianBlur(gray, (25,25), 0)  
    
    # Increase gray mask brightness for non-black pixels only
    # This should circularize the Moon
    gray_blurred[gray_blurred > 1] = 255


    original_median_blurred = cv2.medianBlur(gray_blurred, round_up_to_odd(11))

    up_pad_circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=30, minRadius=1, maxRadius=0)
    if up_pad_circles is not None:
        up_pad_circle = up_pad_circles[0][0]
        x = int((up_pad_circle[0]-padSize)/upsampleFactor) # The detected circle is always a bit off-center
        y = int((up_pad_circle[1]-padSize)/upsampleFactor)
        r = int((up_pad_circle[2])/upsampleFactor)
        # r = int(r * 1.5) # The detected circle is always a bit smaller than is should be
        # x1 = int(up_pad_circle[0])
        # y1 = int(up_pad_circle[1])
        # r1 = int(up_pad_circle[2])
        cv2.circle(image, (x, y), r, (255, 0, 0), 2)
        cv2.circle(image, (x, y), 2, (0, 0, 255), 1)
        # uph, upw, _ = upsampledImage.shape
        return image, x, y, r
    return None

def obtainCesiumMeasurements(path):
    """
    - Given a set of images organized by iterations, this function returns set of measurements
    - removes blank images
    [path]: location of iterations folder
    """
    currIter = 0
    iterPath = os.path.join(path, f'{currIter}')
    print(iterPath)

    def findObjectsInCamera(cam, camPath):
        for filename in os.listdir(camPath):
            imagePath = os.path.join(camPath, filename)
            image = cv2.imread(imagePath)
            if np.min(image) == np.max(image) == 0:
                os.remove(imagePath)
                continue
            stereoImage = rectilinearToStereographicProjection(image)
            moonRes = detectCesiumRedMoon(copy.copy(stereoImage))
            sunRes = detectCesiumSun(copy.copy(stereoImage))
            earthRes = detectCesiumEarth(copy.copy(stereoImage))

            if moonRes is not None:
                (_, moonX, moonY, moonR) = moonRes
                cv2.circle(stereoImage, (moonX, moonY), moonR, (255, 255, 255), 2)
                cv2.circle(stereoImage, (moonX, moonY), 2, (0, 0, 255), 1)
            if earthRes is not None:
                (_, earthX, earthY, earthR) = earthRes
                cv2.circle(stereoImage, (earthX, earthY), earthR, (255, 0, 0), 2)
                cv2.circle(stereoImage, (earthX, earthY), 2, (0, 0, 255), 1)
            if sunRes is not None:
                (_, sunX, sunY, sunR) = sunRes
                cv2.circle(stereoImage, (sunX, sunY), sunR, (0, 255, 255), 2)
                cv2.circle(stereoImage, (sunX, sunY), 2, (0, 0, 255), 1)
            cv2.imshow(f'Stereographic corrected {filename}', stereoImage)
            cv2.waitKey(0)
            cv2.destroyWindow(f'Stereographic corrected {filename}')

    while os.path.exists(iterPath):
        cam1Path = os.path.join(iterPath, f'1')
        cam2Path = os.path.join(iterPath, f'2')
        cam3Path = os.path.join(iterPath, f'3')
        # Check if all camera views exists
        if not os.path.exists(cam1Path) or not os.path.exists(cam2Path) or not os.path.exists(cam3Path):
            continue
        print(iterPath)
        findObjectsInCamera(1, cam1Path)
        findObjectsInCamera(2, cam2Path)
        findObjectsInCamera(3, cam3Path)

        currIter += 1
        iterPath = os.path.join(path, f'{currIter}')

        

if __name__ == "__main__":
    """
    Run "python FilterCesiumSun.py -i=<IMAGE>" to test this module
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--path", help = "path to the iterations folder")
    args = vars(ap.parse_args())

    obtainCesiumMeasurements(args['path'])
    
    # true_image = cv2.imread(args["path"])
    # stereoImage = rectilinearToStereographicProjection(true_image)
    # img = detectCesiumSun(true_image, TEMPLATE_SUN_PATH)  
    # img, _, _, _ = detectCesiumRedMoon(true_image)      
    # img, _, _, _ = detectCesiumRedMoon(stereoImage)    
    # if img is not None:
    #     cv2.imshow("Result Stereographic", img)
    #     cv2.waitKey(0)
    # else:
    #     print("NO circle")

