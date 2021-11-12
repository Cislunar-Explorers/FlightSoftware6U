import cv2
import numpy as np
import itertools
import math


def get_earth_moon_coords(src):
    img = cv2.imread(src)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY)[1]

    # Find contours
    contours = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    if len(contours) == 0:
        print("No contours found")
        return None

    # Get contour with largest area and delete from contours
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    c = contours[max_index]
    del contours[max_index]

    detect_circles(img, c)

    # Determine second largest contour
    overlaps = True
    c2 = None
    while overlaps and len(contours) > 0:

        # Get contour with next largest area
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        c2 = contours[max_index]
        del contours[max_index]

    try:
        detect_circles(img, c2)
        print("Two contours")
    except UnboundLocalError:
        print("One contour")


def detect_circles(img, c):

    # Get four extreme points
    l = tuple(c[c[:, :, 0].argmin()][0])
    r = tuple(c[c[:, :, 0].argmax()][0])
    t = tuple(c[c[:, :, 1].argmin()][0])
    b = tuple(c[c[:, :, 1].argmax()][0])

    # Set of four extreme points
    S = {l, r, t, b}

    # All combinations of three points among set of four extreme points
    S_3 = set(itertools.combinations(S, 3))

    # Get the set of three points that form the largest circle
    sum = 0
    pts = None
    for s in S_3:
        x1, y1 = s[0][0], s[0][1]
        x2, y2 = s[1][0], s[1][1]
        x3, y3 = s[2][0], s[2][1]

        d12 = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        d23 = math.sqrt((x2 - x3) ** 2 + (y2 - y3) ** 2)
        d31 = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)

        if d12 + d23 + d31 >= sum:
            sum = d12 + d23 + d31
            pts = s

    x, y, rad = center_circle(pts)

    print("(%s, %s), radius = %s" % (x, y, rad))


def center_circle(s):
    """center_circle(s) -> (x, y, radius)

    Args:
        s (tuple[tuple, tuple, tuple]): set of three (x,y) tuples

    Returns:
        [type]: [description]
    """

    # Get the three points
    x1, y1 = s[0][0], s[0][1]
    x2, y2 = s[1][0], s[1][1]
    x3, y3 = s[2][0], s[2][1]

    # Calculate distances between points
    x12 = x1 - x2
    x13 = x1 - x3

    y12 = y1 - y2
    y13 = y1 - y3

    y31 = y3 - y1
    y21 = y2 - y1

    x31 = x3 - x1
    x21 = x2 - x1

    # x1^2 - x3^2
    sx13 = pow(x1, 2) - pow(x3, 2)

    # y1^2 - y3^2
    sy13 = pow(y1, 2) - pow(y3, 2)

    # x2^2 - x1^2
    sx21 = pow(x2, 2) - pow(x1, 2)

    # y2^2 - y1^2
    sy21 = pow(y2, 2) - pow(y1, 2)

    f = ((sx13) * (x12) + (sy13) * (x12) + (sx21) * (x13) + (sy21) * (x13)) // (
        2 * ((y31) * (x12) - (y21) * (x13))
    )

    g = ((sx13) * (y12) + (sy13) * (y12) + (sx21) * (y13) + (sy21) * (y13)) // (
        2 * ((x31) * (y12) - (x21) * (y13))
    )

    c = -pow(x1, 2) - pow(y1, 2) - 2 * g * x1 - 2 * f * y1

    # Get x and y position of center
    x = -g
    y = -f

    r2 = x * x + y * y - c

    # Get radius
    r = int(math.sqrt(r2))

    return x, y, r
