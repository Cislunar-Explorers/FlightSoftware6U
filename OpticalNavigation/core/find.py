from core.const import ImageDetectionCircles
import numpy as np
#import scipy as sp
import cv2
import argparse
import copy
import math

EARTH_HSV_BOUNDARIES = [([80,30,1], [160,254,254])]
MOON_HSV_BOUNDARIES = [([0,0,1], [179, 255, 254])]

def sharpen(true_image, kernel_size=3):
    kernel = np.ones((kernel_size,kernel_size))*-1
    kernel[int(kernel_size/2),int(kernel_size/2)] = kernel_size**2
    return cv2.filter2D(true_image, -1, kernel)

def round_up_to_odd(number):
    number = int(np.ceil(number))
    return number + 1 if number % 2 == 0 else number

def findSun(image):
    """
    Detect the Sun using HoughCircleTransform
    [image] - w x h x 3 input image
    Returns:
    [circles] - 1 x n x 3 list of n detected circles
    """
    boundaries = [([0,0,255], [180,51,255])]
    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    sunRemoved = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)

        # Delete the sun pixels to reduce confusion for other bodies
        masknot = cv2.bitwise_not(mask)
        sunRemoved = cv2.bitwise_and(image, image, mask = masknot)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)

    scale = 1
    (c, r) = np.shape(gray)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    # upsamlp, upsampleFactor = ((cv2.pyrUp(gray))), 2
    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(3))

    # TODO: 4th arguement might be too large
    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, round_up_to_odd(700 / 50), param1=80, param2=15, minRadius=0, maxRadius=0)
    return circles, sunRemoved

def findEarth(image):
    """
    Detect the Earth using HoughCircleTransform
    [image] - w x h x 3 input image
    Returns:
    [circles] - 1 x n x 3 list of n detected circles
    """
    boundaries = EARTH_HSV_BOUNDARIES
    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    earthRemoved = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)

        masknot = cv2.bitwise_not(mask)
        earthRemoved = cv2.bitwise_and(image, image, mask = masknot)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)

    # Smudge the gray mask around to fill in missing gaps
    gray_blurred = cv2.GaussianBlur(gray, (11,11), 0)  
    
    black_mask = np.clip( (gray_blurred + gray_blurred/255 * 200).astype('uint8') , a_min = 0, a_max=255)

    scale = 1
    (c, r) = np.shape(black_mask)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(black_mask, round_up_to_odd(700 / 500))

    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=30, minRadius=10, maxRadius=0)
    return circles, earthRemoved

def findMoon(image):
    """
    Detect the Moon using HoughCircleTransform
    [image] - w x h x 3 input image
    Returns:
    [circles] - 1 x n x 3 list of n detected circles
    """
    # TODO: Hard to distinguish between Moon and Sun
    boundaries = MOON_HSV_BOUNDARIES

    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    gray_blurred = cv2.GaussianBlur(gray, (11,11), 0)
    
    # cv2.imshow('Blur', cv2.pyrDown(gray_blurred))
    # cv2.waitKey(0)

    # Increase brightness
    black_mask = np.clip( (gray_blurred + gray_blurred/255 * 10).astype('uint8') , a_min = 0, a_max=255)

    # Hough Transform
    original_median_blurred = cv2.medianBlur(black_mask, round_up_to_odd(700 / 200))

    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=15, minRadius=1, maxRadius=0)
    return circles

def contains_blue(img):
    """https://stackoverflow.com/questions/51324202/check-if-an-image-contains-blue-pixels"""
    # Convert the image to HSV colour space
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Define a range for blue color
    (hsv_l,hsv_h) = EARTH_HSV_BOUNDARIES[0]
    hsv_l = np.array(hsv_l, dtype = "uint8")
    hsv_h = np.array(hsv_h, dtype = "uint8")
    # Find blue pixels in the image
    #
    # cv2.inRange will create a mask (binary array) where the 1 values
    # are blue pixels and 0 values are any other colour out of the blue
    # range defined by hsv_l and hsv_h
    return cv2.inRange(hsv, hsv_l, hsv_h).any()

# https://stackoverflow.com/questions/44865023/how-can-i-create-a-circular-mask-for-a-numpy-array
def create_circular_mask(h, w, center=None, radius=None):

    if center is None: # use the middle of the image
        center = (int(w/2), int(h/2))
    if radius is None: # use the smallest distance between the center and image walls
        radius = min(center[0], center[1], w-center[0], h-center[1])

    Y, X = np.ogrid[:h, :w]
    dist_from_center = np.sqrt((X - center[0])**2 + (Y-center[1])**2)

    mask = dist_from_center <= radius
    return mask

# H: 0-179, S: 0-255, V: 0-255
def findHough(true_image, visualize=False):
    original_with_circles = copy.copy(true_image)
    orig_copy = copy.copy(true_image)
    (origH, origW, origC) = true_image.shape
    
    earthCircle = None
    moonCircle = None
    sunCircle = None

    # We detect sun first because it is the most consistent body visually
    circles, sunRemoved = findSun(true_image)
    if circles is not None:   
        circles = circles[0][:][:]
        sunCircle = circles[0]
        sunCircle[0] = int(sunCircle[0]) 
        sunCircle[1] = int(sunCircle[1]) 
        sunCircle[2] = int(sunCircle[2])
        if visualize:
            for i in range(0,1):
                cv2.circle(original_with_circles, (sunCircle[0], sunCircle[1]), sunCircle[2], (0, 255, 255), 2)
                cv2.circle(original_with_circles, (sunCircle[0], sunCircle[1]), 2, (0, 0, 255), 1)
    
    # Upsample x8
    denoise = cv2.medianBlur(sunRemoved, round_up_to_odd(1))

    true_image, upsampleFactor = (cv2.pyrUp(cv2.pyrUp(denoise))), 2*2
    sharpened_im = sharpen(true_image, kernel_size=3)
    (a, b, c) = np.shape(sharpened_im)

    #We detect Earth next simply because The Moon is the most challenging, and we want eliminate as many possibilites
    circles, earthRemoved = findEarth(sharpened_im)
    if circles is not None:   
        circles = circles[0][:][:]
        earthCircle = circles[0]
        earthCircle[0] = int(earthCircle[0]/upsampleFactor)
        earthCircle[1] = int(earthCircle[1]/upsampleFactor)
        earthCircle[2] = int(earthCircle[2]/upsampleFactor)

    # No need to return moonRemoved as this is the last body
    circles = findMoon(earthRemoved)
    if circles is not None:   
        circles = circles[0][:][:]
        moonCircle = circles[0]
        moonCircle[0] = int(moonCircle[0]/upsampleFactor)
        moonCircle[1] = int(moonCircle[1]/upsampleFactor)
        moonCircle[2] = int(moonCircle[2]/upsampleFactor)
        # Reject collisions between Earth and Moon or Sun and Moon
        if (earthCircle is not None and math.sqrt((moonCircle[0]-earthCircle[0])**2 + (moonCircle[1]-earthCircle[1])**2) < moonCircle[2] + earthCircle[2]):
            searchSpace = [int((moonCircle[0]+earthCircle[0])/2), int((moonCircle[1]+earthCircle[1])/2), int(max(moonCircle[2], earthCircle[2]))]
            top = max(0, searchSpace[1]-searchSpace[2])
            bottom = min(origH, searchSpace[1]+searchSpace[2])
            left = max(0, searchSpace[0]-searchSpace[2])
            right = min(origW, searchSpace[0]+searchSpace[2])  
            if moonCircle[2] < earthCircle[2] or contains_blue(orig_copy[top:bottom, left:right]):
                # print("Collision, rejecting Moon")
                moonCircle = None
            else:
                # print("Collision, rejecting Earth")
                earthCircle = None

    if visualize:
        if earthCircle is not None:
            cv2.circle(original_with_circles, (earthCircle[0], earthCircle[1]), earthCircle[2], (255, 0, 0), 2)
            cv2.circle(original_with_circles, (earthCircle[0], earthCircle[1]), 2, (0, 0, 255), 1)
        if moonCircle is not None:
            cv2.circle(original_with_circles, (moonCircle[0], moonCircle[1]), moonCircle[2], (200, 200, 200), 2)
            cv2.circle(original_with_circles, (moonCircle[0], moonCircle[1]), 2, (0, 0, 255), 1)

    return sunCircle, earthCircle, moonCircle, np.hstack([original_with_circles])
    
def find(img) -> ImageDetectionCircles:
    """
    Locates the Earth, Moon and Sun in image [img] in spherical coordinates.
    @returns
    [circles]: tuple of 3 tuples containing circle center for each body
        Format: ((ex, ey, ez, edia), (mx, my, mz, mdia), (sx, sy, sz, sdia))
        Returns None if a body is not detected instead of its tuple
    """
    # Location of circle center in spherical coordinates,
    # Angular diameter of circle in radians
    sx, sy, sz, sdia = 0., 0., 0., 0. # Sun
    mx, my, mz, mdia = 0., 0., 0., 0. # Moon
    ex, ey, ez, edia = 0., 0., 0., 0. # Earth
    imageDetectionCircles = ImageDetectionCircles()
    imageDetectionCircles.set_earth_detection(ex, ey, ez, edia)
    imageDetectionCircles.set_moon_detection(mx, my, mz, mdia)
    imageDetectionCircles.set_earth_detection(sx, sy, sz, sdia)
    return imageDetectionCircles

if __name__ == "__main__":
    """
    Run "python3 find.py -i=<IMAGE>" to test this module
    """
    print("[OPNAV]: Let's test out find.py")
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help = "path to the image")
    args = vars(ap.parse_args())
    
    true_image = cv2.imread(args["image"])
    _, _, _, img = find(true_image, visualize=True)    
    cv2.imshow("Result", img)
    cv2.waitKey(0)