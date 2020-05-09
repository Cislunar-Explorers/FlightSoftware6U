import numpy as np
#import scipy as sp
import cv2
import argparse
import copy

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
        # cv2.imshow("sun removed", sunRemoved) #delete 'image' for post mask image
        # cv2.waitKey(0)


    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    scale = 1
    (c, r) = np.shape(gray)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(rows / 50))
    # TODO: 4th arguement might be too large
    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, round_up_to_odd(rows / 50), param1=80, param2=15, minRadius=1, maxRadius=0)
    return circles, sunRemoved

def findEarth(image):
    """
    Detect the Earth using HoughCircleTransform
    [image] - w x h x 3 input image
    Returns:
    [circles] - 1 x n x 3 list of n detected circles
    """
    boundaries = [([80,30,0], [160,255,255])]
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
        # cv2.imshow("earth removed", earthRemoved) #delete 'image' for post mask image
        # cv2.waitKey(0)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    # cv2.imshow("Grayscale - Earth", gray)
    # cv2.waitKey(0)
    # Make all non-black pixels white
    lower_black = np.array([2], dtype = "uint16")
    upper_black = np.array([255], dtype = "uint16")
    black_mask = cv2.inRange(gray, lower_black, upper_black)
    # cv2.imshow('Whiten-Earth',black_mask)
    # cv2.waitKey(0)
    #
    scale = 1
    (c, r) = np.shape(black_mask)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(black_mask, round_up_to_odd(rows / 1000))
    # cv2.imshow("Median Blur - Earth", original_median_blurred)
    # cv2.waitKey(0)
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
    boundaries = [([0,0,1], [179, 25, 254])]
    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)
        # cv2.imshow("moon images", np.hstack([output])) #delete 'image' for post mask image
        # cv2.waitKey(0)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    cv2.imshow("Grayscale - Earth", gray)
    cv2.waitKey(0)
    # scale this up first
    # scale_percent = 100 # percent of original size
    # width = int(gray.shape[1] * scale_percent / 100)
    # height = int(gray.shape[0] * scale_percent / 100)
    # dim = (width, height)
    # resized = cv2.resize(gray, dim, interpolation = cv2.INTER_AREA)
    # cv2.imshow("Resized image", resized)
    # cv2.waitKey(0)
    #
    # Boost whiteness
    lower_black = np.array([1], dtype = "uint16")
    upper_black = np.array([255], dtype = "uint16")
    black_mask = cv2.inRange(gray, lower_black, upper_black)
    scale = 1
    (c, r) = np.shape(black_mask)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(black_mask, round_up_to_odd(rows / 500))
    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=15, minRadius=1, maxRadius=0)
    return circles

def main():
    """
    Run "python3 find.py -i=<IMAGE>" to test this module
    """
    print("[OPNAV]: Let's test out find.py")
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help = "path to the image")
    args = vars(ap.parse_args())
    
    true_image = cv2.imread(args["image"])
    image = copy.copy(true_image)
    original_with_circles = copy.copy(true_image)

    # We detect sun first because it is the most consistent body visually
    circles, sunRemoved = findSun(image)
    print("------\nSUN\n")
    print(circles)
    if circles is None: 
        print("No Circles Detected!")
    else:
        circles = circles[0][:][:]
        for i in range(0,1):
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (0, 255, 255), 5)
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    # We detect Earth next simply because The Moon is the most challenging, and we want eliminate as many possibilites
    # image = copy.copy(true_image)
    circles, earthRemoved = findEarth(sunRemoved)
    print("------\nEarth\n")
    print(circles)
    if circles is None: 
        print("No Circles Detected!")
    else:
        circles = circles[0][:][:]
        for i in range(0,1):
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (255, 0, 0), 5)
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    # No need to return moonRemoved as this is the last body
    # image = copy.copy(true_image)
    circles = findMoon(earthRemoved)
    print("------\nMoon\n")
    print(circles)
    if circles is None: 
        print("No Circles Detected!")
    else:
        circles = circles[0][:][:]
        for i in range(0,1):
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (200, 200, 200), 5)
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    cv2.imshow("Result", np.hstack([original_with_circles]))
    cv2.waitKey(0)

if __name__ == "__main__":
    main()    

       
