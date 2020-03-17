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
    [circles] - list of detected circles
    """
    boundaries = [([0,0,255], [180,51,255])]
    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)
        cv2.imshow("sun images", np.hstack([output])) #delete 'image' for post mask image
        cv2.waitKey(0)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    scale = 1
    (c, r) = np.shape(gray)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(rows / 50))
    # TODO: 4th arguement might be too large
    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, round_up_to_odd(rows / 50), param1=80, param2=15, minRadius=1, maxRadius=0)
    return circles

def findEarth(image):
    """
    Detect the Earth using HoughCircleTransform
    [image] - w x h x 3 input image
    Returns:
    [circles] - list of detected circles
    """
    boundaries = [([80,30,0], [160,255,255])]
    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)
        cv2.imshow("earth images", np.hstack([output])) #delete 'image' for post mask image
        cv2.waitKey(0)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    scale = 1
    (c, r) = np.shape(gray)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(rows / 50))
    # TODO: 4th arguement might be too large
    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=30, minRadius=10, maxRadius=0)
    return circles

def findMoon(image):
    """
    Detect the Moon using HoughCircleTransform
    [image] - w x h x 3 input image
    Returns:
    [circles] - list of detected circles
    """
    boundaries = [([0,0,0], [179, 25, 254])]
    original_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    output = None
    # Note: Calculates mask for one boundary only
    for lower,upper in boundaries:
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")

        mask = cv2.inRange(original_hsv, lower, upper)
        output = cv2.bitwise_and(original_hsv, original_hsv, mask = mask)
        cv2.imshow("moon images", np.hstack([output])) #delete 'image' for post mask image
        cv2.waitKey(0)

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)
    scale = 1
    (c, r) = np.shape(gray)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(gray, round_up_to_odd(rows / 50))
    # TODO: 4th arguement might be too large
    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 100, param1=400, param2=30, minRadius=1, maxRadius=0)
    return circles

def runFind():
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
    circles = findSun(image)
    print("------\nSUN\n")
    print(circles)
    if circles is None: 
        print("No Circles Detected!")
    else:
        circles = circles[0][:][:]
        for i in range(0,1):
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (0, 255, 255), 5)
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    image = copy.copy(true_image)
    circles = findEarth(image)
    print("------\nEarth\n")
    print(circles)
    if circles is None: 
        print("No Circles Detected!")
    else:
        circles = circles[0][:][:]
        for i in range(0,1):
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (255, 0, 0), 5)
            cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    image = copy.copy(true_image)
    circles = findMoon(image)
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
    runFind()    

       
