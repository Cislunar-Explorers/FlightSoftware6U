import cv2
import numpy as np
import scipy as sp
import pandas as pd
import matplotlib.pyplot as plt
import os

def get_earth_moon_coords(src):
    img = cv2.imread(src)
    
    # plt.imshow(img)
    # plt.show()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)[1]

    # plt.imshow(thresh)
    # plt.show()
    
    #Find contours
    contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    
    if len(contours) == 0:
        print('No contours found')
        return None
    
    #Get contour with largest area and delete from contours
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    c = contours[max_index]
    del contours[max_index]
    
    #Draw contour
    #cv2.drawContours(img, np.array(c), -1, (0,255,0), 3)
    x, y, w, h = cv2.boundingRect(c)
    contour_one = thresh[y:y+h,x:x+w]
    og_one = img[y:y+h,x:x+w]
    detect_circles(og_one, contour_one)
    
    #Determine second largest contour
    overlaps = True
    while overlaps and len(contours) > 0:

        #Get contour with next largest area
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        c2 = contours[max_index]
        del contours[max_index]

        #cv2.drawContours(img, np.array(c2), -1, (0,255,0), 3)
        x2, y2, w2, h2 = cv2.boundingRect(c2)
    
        #Check whether contour overlaps
        overlaps = False
        if (x2 > x and x2 < x + w and y2 > y and y2 < y + h) or (x2 + w2 > x and x2 + w2 < x + w and y2 > y and y2 < y + h) or (x2 + w2 > x and x2 + w2 < x + w and y2 + h2 > y and y2 + h2 < y + h) or (x2 > x and x2 < x + w and y2 + h2 > y and y2 + h2 < y + h):
            overlaps = True
        
        og_two = img[y2:y2+h2,x2:x2+w2]
        contour_two = thresh[y2:y2+h2,x2:x2+w2]

    try:
      detect_circles(og_two, contour_two)
      print("Two contours")
    except UnboundLocalError:
      print("One contour")

def detect_circles(og, img):

    dim = (256, int((256 / img.shape[1]) * img.shape[0]))

    if img.shape[1] > 256:
      og_sml = cv2.resize(og, dim)
      sml = cv2.resize(img, dim)
    else:
      og_sml = og.copy()
      sml = img.copy()

    output = og_sml.copy()

    # detect circles in the image
    circles = cv2.HoughCircles(sml, cv2.HOUGH_GRADIENT, 1, sml.shape[0],
                               param1=100, param2=10,
                               minRadius=int(sml.shape[0] / 8), maxRadius=sml.shape[0]*2)
    # ensure at least some circles were found
    if circles is not None:
	      # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            print("(%s, %s), radius = %s" % (x, y, r))
            # cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            # cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        # plt.imshow(output)
        # plt.show()