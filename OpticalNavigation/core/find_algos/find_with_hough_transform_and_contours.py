from core.const import (
    ImageDetectionCircles,
    CameraParameters,
    CisLunarCameraParameters,
    BodyEnum,
)
from utils.parameters import (
    EARTH_B_LOW,
    EARTH_G_LOW,
    EARTH_R_LOW,
    MOON_B_LOW,
    MOON_G_LOW,
    MOON_R_LOW,
    SUN_B_LOW,
    SUN_G_LOW,
    SUN_R_LOW,
    EARTH_B_HIGH,
    EARTH_G_HIGH,
    EARTH_R_HIGH,
    MOON_B_HIGH,
    MOON_G_HIGH,
    MOON_R_HIGH,
    SUN_B_HIGH,
    SUN_G_HIGH,
    SUN_R_HIGH,
    EARTH_PERCENTAGE_THRESH,
    SUN_PERCENTAGE_THRESH,
    MOON_PERCENTAGE_THRESH,
)
from OpticalNavigation.core.find_algos.tiled_remap import *
import cv2
import numpy as np
from math import radians
import re
import logging

import argparse


def drawContourCircle(img, xy, r, contours):
    """
    Draws the contour and the minimum enclosing circle on the original image
    """
    x = int(xy[0])
    y = int(xy[1])
    r = int(r)
    cv2.drawContours(img, contours[0], -1, (0, 255, 0), 1)
    cv2.circle(img, (x, y), r, (255, 0, 0), 1)
    cv2.imshow("name", img)
    cv2.waitKey(0)


def __findMinEnclosingCircle(img, highThresh, showCircle=False):
    """
    Finds the minimum enclosing circle of the contours found on the image. This is the old algorithm for finding center
    """
    contours = cv2.findContours(highThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]
    if len(contours) != 0:
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        c = contours[max_index]
        xy, r = cv2.minEnclosingCircle(c)
        if showCircle:
            drawContourCircle(img, xy, r, contours)
        return xy, r
    return None


def circleArea(circle):
    """
    Returns the area of a circle.
    """
    r = circle[2]
    return np.pi * r ** 2


def __houghCircleWithContour(img, w, h, highThresh, showCircle):
    """
    An algorithm that finds the circle center that uses the hough transform first with the max radius being half of the
    minimum of the width/height of the bounding box to improve performance. If hough transform does not find anything,
    it then turns to contour finding.
    """
    maxRadius = min(w, h)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        1,
        120,
        param1=10,
        param2=20,
        minRadius=0,
        maxRadius=int(maxRadius / 2) + 1,
    )
    if circles is None:
        return __findMinEnclosingCircle(img, highThresh, showCircle)
    areas = [circleArea(circle) for circle in circles[0]]
    max_idx = np.argmax(areas)
    circle = circles[0][max_idx]
    xy = (int(circle[0]), int(circle[1]))
    r = int(circle[2])
    if showCircle:
        cv2.circle(img, xy, r, (255, 255, 255), 3)
        cv2.imshow("name", img)
        cv2.waitKey(0)
    return xy, r


def __findBody(img, thresh, body, w, h, showCircle=False):
    """
    Finds the earth, sun, moon
    """
    percentWhite = cv2.countNonZero(thresh) / (thresh.shape[0] * thresh.shape[1])
    # print(body + str(percentWhite))
    if body == "e":
        percentageThresh = EARTH_PERCENTAGE_THRESH
        # highThreshRed = cv2.inRange(img, (0, 0, 5), (255, 255, 255))
        # highThreshGreen = cv2.inRange(img, (0, 5, 0), (255, 255, 255))
        # highThreshBlue = cv2.inRange(img, (5, 0, 0), (255, 255, 255))
        # highThresh = highThreshRed + highThreshGreen + highThreshBlue
    elif body == "s":
        percentageThresh = SUN_PERCENTAGE_THRESH
        # highThresh = cv2.inRange(img, (225, 225, 225), (255, 255, 255))
    else:
        percentageThresh = MOON_PERCENTAGE_THRESH
        # highThresh = thresh
    if percentWhite >= percentageThresh:
        # TODO: Shift center based on aspect ratio, center of "mass"
        # return __findMinEnclosingCircle(img, thresh, showCircle)
        return __houghCircleWithContour(img, w, h, thresh, showCircle)


# Threshold based primarily on blue and green channels
# Percent of white pixels determines if earth detected
def measureEarth(img, w, h):
    EARTH_THRESH = (
        (EARTH_B_LOW, EARTH_G_LOW, EARTH_R_LOW),
        (EARTH_B_HIGH, EARTH_G_HIGH, EARTH_R_HIGH),
    )
    return __findBody(
        img, cv2.inRange(img, EARTH_THRESH[0], EARTH_THRESH[1]), "e", w, h
    )


# Measure white pixels
def measureSun(img, w, h):
    SUN_THRESH = (
        (SUN_B_LOW, SUN_G_LOW, SUN_R_LOW),
        (SUN_B_HIGH, SUN_G_HIGH, SUN_R_HIGH),
    )
    return __findBody(img, cv2.inRange(img, SUN_THRESH[0], SUN_THRESH[1]), "s", w, h)


# TODO add code for % white, but parameterize if we can to use it
def measureMoon(img, w, h):
    MOON_THRESH = (
        (MOON_B_LOW, MOON_G_LOW, MOON_R_LOW),
        (MOON_B_HIGH, MOON_G_HIGH, MOON_R_HIGH),
    )
    return __findBody(img, cv2.inRange(img, MOON_THRESH[0], MOON_THRESH[1]), "m", w, h)


def find(
    src,
    camera_params: CameraParameters = CisLunarCameraParameters,
    st=False,
    pixel=True,
):
    cam = Camera(
        radians(camera_params.hFov),
        radians(camera_params.vFov),
        camera_params.hPix,
        camera_params.vPix,
    )

    # u is in body frame here
    # Assumes only spinning about y-axis
    u = np.array([0, 1, 0], dtype=np.float32)
    camNum = int(re.search(r"[cam](\d+)", src).group(1))
    assert camNum in [1, 2, 3], "Error in getting the camera number from filename"
    if camNum == 1:
        cam_DCM: np.ndarray = camera_params.cam1Rotation.T
        u = cam_DCM.dot(u)
    elif camNum == 2:
        cam_DCM: np.ndarray = camera_params.cam2Rotation.T
        u = cam_DCM.dot(u)
    elif camNum == 3:
        cam_DCM: np.ndarray = camera_params.cam3Rotation.T
        u = cam_DCM.dot(u)
    # u is now in the camera frame
    # TODO switch to gyro database
    omega = -5
    # Single row readout time
    dt = 18.904e-6
    rot = CameraRotation(u, -omega * dt)

    result = ImageDetectionCircles()

    img = cv2.imread(src)
    # In-place blur to reduce noise, avoid hot pixels
    img = cv2.GaussianBlur(img, (5, 5), 0, dst=img)

    # Extract and threshold channels
    # TODO make all thresholds parameters
    bwThreshRed = cv2.inRange(img, (0, 0, 50), (255, 255, 255))
    bwThreshGreen = cv2.inRange(img, (0, 50, 0), (255, 255, 255))
    bwThreshBlue = cv2.inRange(img, (50, 0, 0), (255, 255, 255))
    bw = bwThreshRed + bwThreshGreen + bwThreshBlue

    contours = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Hack around API breakage between OpenCV versions
    contours = contours[0] if len(contours) == 2 else contours[1]
    if len(contours) == 0:
        logging.debug("[OPNAV]: No contours found")
        return result, {}

    areas = [cv2.contourArea(c) for c in contours]
    areas = np.array(areas)
    largest_indices = (-areas).argsort()[
        :2
    ]  # Gets the largest two indices of areas array
    max_index = largest_indices[0]
    second_index = largest_indices[1] if largest_indices.size > 1 else None
    # max_index = np.argmax(areas)
    c1 = contours[max_index]
    c2 = contours[second_index] if second_index is not None else None
    # del contours[max_index]
    # print(f"Second Index: {second_index}")

    x, y, w, h = cv2.boundingRect(c1)
    x, y, w, h = bufferedRoi(x, y, w, h, cam.w, cam.h, 16)
    if not st:
        box = BoundingBox(x, y, w, h)
        out, bbst = remap_roi(img, box, cam, rot)
        x = bbst.x0
        y = bbst.y0
    else:
        bbst = BoundingBox(x, y, w, h)
        out = img[y : y + h, x : x + w]
    # TODO make sure we can handle eclipse edge case
    # Gets the next largest body that doesn't overlap with first body
    # c2 = None
    x2, y2, w2, h2 = 0, 0, 0, 0
    out2 = None
    bbst2 = BoundingBox(0, 0, 0, 0)

    if c2 is not None and cv2.contourArea(c2) >= 100:  # TODO: Placeholder
        x2, y2, w2, h2 = cv2.boundingRect(c2)
        # Checks for rectangle overlap
        if x + w < x2 or x > x2 + w2 or y < y2 + h2 or y2 + h2 > y:
            # c2 = con
            x2, y2, w2, h2 = bufferedRoi(x2, y2, w2, h2, cam.w, cam.h, 16)
            box2 = BoundingBox(x2, y2, w2, h2)
            out2, bbst2 = remap_roi(img, box2, cam, rot)
        else:
            x2, y2, w2, h2 = 0, 0, 0, 0

    # Measure body in region-of-interest
    sun = None
    earth = None
    moon = None
    body_values = {}
    if "Low" in src:
        for f in [out]:
            sun = measureSun(f, w, h)
            if sun is not None:
                (sX, sY), sR = sun
                sXst, sYst = cam.normalize_st(bbst.x0 + sX, bbst.y0 + sY)
                sRho2 = sXst ** 2 + sYst ** 2
                sRho = np.sqrt(sRho2)
                # TODO figure out if below line (and corresponding ones) is diameter in stereographic or spherical
                sDia = 4 * 2 * sR * (2 * cam.xmax_st / cam.w) / (4 + sRho2)
                sSx, sSy, sSz = st_to_sph(sXst, sYst)
                result.set_sun_detection(sSx, sSy, sSz, sDia)

                sAngDiam = get_angular_size(sRho, sR, cam.st_scale)

                # logging.debug(f"File: {src}")
                # logging.debug(f"Sun sX: {sX} sY: {sY} sR: {sR}")
                # logging.debug(f"Sun sXst: {sXst} sYst: {sYst}")
                # logging.debug(f"sAngDiam: {sAngDiam}\n")

                # Andrew
                if pixel:
                    body_values[BodyEnum.Sun] = [x + sX - 1640, y + sY - 1232, sR]
                else:
                    body_values[BodyEnum.Sun] = [
                        cam.normalize_st(x + sX, y + sY)[0],
                        cam.normalize_st(x + sX, y + sY)[1],
                        sAngDiam,
                    ]

    else:
        earth = None
        index = 0
        for f in [out, out2]:
            if f is None:
                break
            # if f is None or cv2.sumElems(f) == (0, 0, 0, 0) or measureSun(f, w, h) is not None:
            #     continue
            if earth is not None:
                earth = None
            elif np.max(areas) > 400 and index == 0:  # TODO: Placeholder
                earth = measureEarth(f, w, h)
            if earth is not None:
                (eX, eY), eR = earth
                eXst, eYst = cam.normalize_st(bbst.x0 + eX, bbst.y0 + eY)
                eRho2 = eXst ** 2 + eYst ** 2
                eRho = np.sqrt(eRho2)
                eDia = 4 * 2 * eR * (2 * cam.xmax_st / cam.w) / (4 + eRho2)
                eSx, eSy, eSz = st_to_sph(eXst, eYst)
                result.set_earth_detection(eSx, eSy, eSz, eDia)

                eAngDiam = get_angular_size(eRho, eR, cam.st_scale)

                # logging.debug(f"File: {src}")
                # logging.debug(f"Earth eX: {eX} eY: {eY} eR: {eR}")
                # logging.debug(f"Earth eXst: {eXst} eYst: {eYst}")
                # logging.debug(f"eAngDiam: {eAngDiam}\n")

                # Andrew
                if pixel:
                    body_values[BodyEnum.Earth] = [x + eX - 1640, y + eY - 1232, eR]
                else:
                    body_values[BodyEnum.Earth] = [
                        cam.normalize_st(x + eX, y + eY)[0],
                        cam.normalize_st(x + eX, y + eY)[1],
                        eAngDiam,
                    ]
            elif earth is None:  # TODO: Placeholder
                moon = measureMoon(f, w, h)
                if moon is not None:
                    (mX, mY), mR = moon
                    mXst, mYst = None, None
                    # Checks whether moon contour is first or second contour
                    if index == 0:
                        mXst, mYst = cam.normalize_st(bbst.x0 + mX, bbst.y0 + mY)
                    elif index == 1:
                        mXst, mYst = cam.normalize_st(bbst2.x0 + mX, bbst2.y0 + mY)
                    else:
                        mXst, mYst = cam.normalize_st(bbst.x0 + mX, bbst.y0 + mY)
                    mRho2 = mXst ** 2 + mYst ** 2
                    mRho = np.sqrt(mRho2)
                    mDia = 4 * 2 * mR * (2 * cam.xmax_st / cam.w) / (4 + mRho2)
                    mSx, mSy, mSz = st_to_sph(mXst, mYst)
                    result.set_moon_detection(mSx, mSy, mSz, mDia)

                    mAngDiam = get_angular_size(mRho, mR, cam.st_scale)

                    # logging.debug(f"File: {src}")
                    # logging.debug(f"Moon mX: {mX} mY: {mY} mR: {mR}")
                    # logging.debug(f"Moon mXst: {mXst} mYst: {mYst}")
                    # logging.debug(f"mAngDiam: {mAngDiam}\n")

                    # Andrew
                    if pixel:
                        body_values[BodyEnum.Moon] = [x + mX - 1640, y + mY - 1232, mR]
                    else:  # Checks whether moon contour is first or second contour
                        if index == 1:
                            body_values[BodyEnum.Moon] = [
                                cam.normalize_st(x + mX, y + mY)[0],
                                cam.normalize_st(x + mX, y + mY)[1],
                                mAngDiam,
                            ]
                        else:
                            body_values[BodyEnum.Moon] = [
                                cam.normalize_st(x2 + mX, y2 + mY)[0],
                                cam.normalize_st(x2 + mX, y2 + mY)[1],
                                mAngDiam,
                            ]

            index += 1
    return result, body_values


# Shift stereographic coordinates of center to camera frame (at start of exposure)

# Adjust diameter for stereographic distortion

# Compute spherical coordinates of center (in camera frame at start of exposure)

# Location of circle center in spherical coordinates,
# Angular diameter of circle in radians


# Notes
# * Need sanity check on contour size (not too large, not too small)
# * Need sanity check on omega (too high and out will be too big)
# * Need to characterize, avoid sun flare in planet exposures
# * Sun is still too bright even with min exposure
#   * Can white balance, gamma reduce sensitivity?
#   * Can we install ND filters (and how would they affect planet exposures)?
#   * Note: Can measure center quite well, just not size; is it required?  Could we fudge it with a correction factor?
# * For Moon, blurry edges and non-uniform surface make picking threshold value difficult
#   * Consider analyzing ROI of original, unblurred image
# * Bottleneck on RPi appears to be finding contours on full-res image.  Consider finding contours on low-res image,
#   then scaling ROI
if __name__ == "__main__":
    """
    Run "python3 find_with_contours.py -i=<IMAGE>" to test this module
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--image", help="path to the image")
    args = vars(ap.parse_args())
    result, body_values = find(args["image"])
    logging.debug(body_values)
    print(f"Body_Values:\n{body_values}")
# Notes
# * Need sanity check on contour size (not too large, not too small)
# * Need sanity check on omega (too high and out will be too big)
# * Need to characterize, avoid sun flare in planet exposures
# * Sun is still too bright even with min exposure
#   * Can white balance, gamma reduce sensitivity?
#   * Can we install ND filters (and how would they affect planet exposures)?
#   * Note: Can measure center quite well, just not size; is it required?  Could we fudge it with a correction factor?
# * For Moon, blurry edges and non-uniform surface make picking threshold value difficult
#   * Consider analyzing ROI of original, unblurred image
# * Bottleneck on RPi appears to be finding contours on full-res image.  Consider finding contours on low-res image,
#   then scaling ROI
