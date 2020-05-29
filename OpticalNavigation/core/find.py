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

    gray = cv2.cvtColor(cv2.cvtColor(output, cv2.COLOR_HSV2BGR), cv2.COLOR_BGR2GRAY)

    # Make all non-black pixels white
    lower_black = np.array([2], dtype = "uint16")
    upper_black = np.array([255], dtype = "uint16")
    black_mask = cv2.inRange(gray, lower_black, upper_black)

    #
    scale = 1
    (c, r) = np.shape(black_mask)
    rows = int(np.round(r * scale))
    cols = int(np.round(c * scale))
    original_median_blurred = cv2.medianBlur(black_mask, round_up_to_odd(rows / 1000))

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

    # Canny edge detector
    sigma = 0.1
    # function to process image with canny method with user chosen thresholds
    def auto_canny(image, sigma):
        v = np.median(image)

        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(max(255, (1.0 + sigma) * v))
        edged = cv2.Canny(image, lower, upper)

        return edged
    gaussian = cv2.GaussianBlur(black_mask, (101, 101), round_up_to_odd(rows / 100))
    auto_canny = auto_canny(gaussian, sigma)

    # Hough Transform

    original_median_blurred = cv2.medianBlur(black_mask, round_up_to_odd(rows / 500))

    circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 2, 50, param1=80, param2=15, minRadius=1, maxRadius=0)
    return circles

def find(true_image, visualize=False):
    (a, b, c) = np.shape(true_image)
    a = int(np.round(a * 1))
    b = int(np.round(b * 1))
    true_image = cv2.resize(true_image, (a,b))
    image = copy.copy(true_image)
    original_with_circles = copy.copy(true_image)

    earthCircle = None
    moonCircle = None
    sunCircle = None

    # We detect sun first because it is the most consistent body visually
    circles, sunRemoved = findSun(image)
    if circles is not None:   
        circles = circles[0][:][:]
        sunCircle = circles[0]
        if visualize:
            for i in range(0,1):
                cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (0, 255, 255), 5)
                cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    # We detect Earth next simply because The Moon is the most challenging, and we want eliminate as many possibilites
    # image = copy.copy(true_image)
    circles, earthRemoved = findEarth(sunRemoved)
    if circles is not None:   
        circles = circles[0][:][:]
        earthCircle = circles[0]
        if visualize:
            for i in range(0,1):
                cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (255, 0, 0), 5)
                cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)

    # No need to return moonRemoved as this is the last body
    # image = copy.copy(true_image)
    circles = findMoon(earthRemoved)
    if circles is not None:   
        circles = circles[0][:][:]
        moonCircle = circles[0]
        if visualize:
            for i in range(0,1):
                cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), circles[i][2], (200, 200, 200), 5)
                cv2.circle(original_with_circles, (circles[i][0], circles[i][1]), 2, (0, 0, 255), 1)
    if visualize:
        cv2.imshow("Result", np.hstack([original_with_circles]))
        cv2.waitKey(0)
    return sunCircle, earthCircle, moonCircle
    
if __name__ == "__main__":
    """
    Run "python3 find.py -i=<IMAGE>" to test this module
    """
    print("[OPNAV]: Let's test out find.py")
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help = "path to the image")
    args = vars(ap.parse_args())
    
    true_image = cv2.imread(args["image"])
    find(true_image, visualize=True)    

#############################
# import numpy as np
# import scipy as sp
# import cv2
# import matplotlib.pyplot as plt
# import matplotlib.image as mpimg
# import os
# import copy
# import argparse


# # function to process image with canny method with user chosen thresholds
# def auto_canny(image, sigma):
#     v = np.median(image)

#     lower = int(max(0, (1.0 - sigma) * v))
#     upper = int(max(255, (1.0 + sigma) * v))
#     edged = cv2.Canny(image, lower, upper)

#     return edged

# # function rounds a floating point number up to the next odd integer


# def round_up_to_odd(number):
#     number = int(np.ceil(number))
#     return number + 1 if number % 2 == 0 else number


# # read an image from image sub-directory

# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--image", help = "path to the image")
# args = vars(ap.parse_args())

# original = cv2.imread(args["image"])

# # name = "CrescentEarthwithMoon"
# # address_in = "Pictures/" + name + ".jpg"
# # original = cv2.imread(address_in)
# b, g, r = cv2.split(original)
# original_gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
# # good value for SunandEarth 210
# # good value for SunLensFlare 150
# # original_gray[original_gray < 150] = 0

# # scale an image
# scale = 1
# (c, r) = np.shape(original_gray)
# rows = int(np.round(r * scale))
# cols = int(np.round(c * scale))

# original_gaussian_blurred = cv2.GaussianBlur(original_gray, (101, 101), round_up_to_odd(rows / 100))
# original_gaussian_blurred_blue = cv2.GaussianBlur(b, (101, 101), round_up_to_odd(rows / 100))
# # good value 35
# # good value for sun 20
# # good value for small crescent earth
# original_median_blurred = cv2.medianBlur(original_gray, round_up_to_odd(rows / 50))
# original_median_blurred_blue = cv2.medianBlur(b, round_up_to_odd(rows / 80))
# sigma = 0.1
# auto_canny = auto_canny(original_gray, sigma)
# orig_canny = cv2.Canny(original_gray, 270, 280)

# # detect circles
# circles = cv2.HoughCircles(original_median_blurred, cv2.HOUGH_GRADIENT, 1, 150, param1=200, param2=20, minRadius=0, maxRadius=0)

# # plot circles on original image
# original_with_circles = copy.copy(original)
# if circles is None:
#     print("No Circles Detected!")
# else:
#     for i in circles[0, :]:
#         cv2.circle(original_with_circles, (i[0], i[1]), i[2], (0, 255, 0), 2)
#         cv2.circle(original_with_circles, (i[0], i[1]), 2, (0, 0, 255), 3)

# # show image
# fig = plt.figure()
# orig = fig.add_subplot(2, 3, 1)
# plt.imshow(cv2.resize(original, (rows, cols))[..., ::-1])
# orig.set_title("Original")
# orig_gray = fig.add_subplot(2, 3, 2)
# plt.imshow(cv2.resize(original_gray, (rows, cols)), cmap='gray')
# orig_gray.set_title("Gray Scale")
# orig_gauss_blur = fig.add_subplot(2, 3, 3)
# plt.imshow(cv2.resize(original_gaussian_blurred, (rows, cols)), cmap='gray')
# orig_gauss_blur.set_title("Gaussian Blur")
# orig_med_blur = fig.add_subplot(2, 3, 4)
# plt.imshow(cv2.resize(original_median_blurred, (rows, cols)), cmap='gray')
# orig_med_blur.set_title("Median Blur")
# can = fig.add_subplot(2, 3, 5)
# plt.imshow(cv2.resize(orig_canny, (rows, cols)), cmap='gray')
# can.set_title("Canny")
# circ = fig.add_subplot(2, 3, 6)
# plt.imshow(cv2.resize(original_with_circles, (rows, cols))[..., ::-1])
# circ.set_title("circles")

# # show all 3 rgb layers of original image
# rescaled_original = cv2.resize(original, (rows, cols))[..., ::-1]
# red_original = copy.copy(rescaled_original)
# green_original = copy.copy(rescaled_original)
# blue_original = copy.copy(rescaled_original)
# fig2 = plt.figure()
# red = fig2.add_subplot(1, 3, 1)
# red_original[:, :, (1, 2)] = 0
# plt.imshow(red_original)
# red.set_title("Red Channel")
# green = fig2.add_subplot(1, 3, 2)
# green_original[:, :, (0, 2)] = 0
# plt.imshow(green_original)
# green.set_title("Green Channel")
# blue = fig2.add_subplot(1, 3, 3)
# blue_original[:, :, (0, 1)] = 0
# plt.imshow(b)
# blue.set_title("Blue Channel")

# plt.show()

# # store image
# # address_out = "Analysis/" + name + "_with_Circles.jpg"
# # cv2.imwrite(address_out, original_with_circles)
       
