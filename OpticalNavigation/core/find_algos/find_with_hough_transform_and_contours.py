from OpticalNavigation.core.const import (
    ImageDetectionCircles,
    CameraParameters,
    CisLunarCameraParameters,
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
from OpticalNavigation.core.const import ImageDetectionCircles, CameraParameters, CisLunarCameraParameters
from dataclasses import dataclass
import cv2
import numpy as np
from math import pi, radians, tan, floor, ceil
import argparse
import re
from utils.log import get_log
import time
import json
import csv
import os

logger = get_log()

class Camera:
    """Encapsulate field-of-view and resolution of a camera."""

    def __init__(self, hfov, vfov, w, h):
        """Construct a new Camera.

        hfov: Horizontal field-of-view [rad]
        vfov: Vertical field-of-view [rad]
        w: Width of image [px]
        h: Height of image [px]
        """

        # Horizontal field-of-view [rad]
        self.hfov = hfov
        # Vertical field-of-view [rad]
        self.vfov = vfov

        # Resolution [px]
        self.w = w
        self.h = h

        # Maximum stereographic x coordinate
        self.xmax_st = 2 * tan(hfov / 4)
        self.ymax_st = 2 * tan(vfov / 4)
        # Maximum gnomonic x coordinate
        self.xmax_gn = tan(hfov / 2)
        self.ymax_gn = tan(vfov / 2)

    def normalize_st(self, xp, yp):
        """Convert pixel coordinates to stereographic coordinates."""
        x = self.xmax_st * (2 * xp / (self.w - 1) - 1)
        y = self.ymax_st * (2 * yp / (self.h - 1) - 1)
        return x, y

    def normalize_gn(self, xp, yp):
        """Convert pixel coordinates to gnomonic coordinates."""
        x = self.xmax_gn * (2 * xp / (self.w - 1) - 1)
        y = self.ymax_gn * (2 * yp / (self.h - 1) - 1)
        return x, y

    def st_to_px(self, x, y):
        """Convert stereographic coordinates to pixel coordinates."""
        xp = (x / self.xmax_st + 1) * (self.w - 1) / 2
        yp = (y / self.ymax_st + 1) * (self.h - 1) / 2
        return xp, yp


def gn_to_sph(xx, yy):
    """Convert gnomonic coordinates to spherical coordinates."""
    zm = np.reciprocal(np.sqrt(np.add.outer(yy ** 2 + 1, xx ** 2)))
    s = np.empty((len(yy), len(xx), 3), dtype=np.float32)
    s[:, :, 0] = xx * zm
    s[:, :, 1] = yy[np.newaxis].T * zm
    s[:, :, 2] = -zm
    return s


def sph_to_st(s):
    """Convert spherical coordinates to stereographic coordinates"""
    x = 2 * s[:, :, 0] / (1 - s[:, :, 2])
    y = 2 * s[:, :, 1] / (1 - s[:, :, 2])
    return x, y


def st_to_sph(x, y):
    """Convert stereographic coordinates to spherical coordinates."""
    norm = x ** 2 + y ** 2 + 4
    # Added a negative sign to z value below
    return 4 * x / norm, 4 * y / norm, -(norm - 8) / norm


@dataclass
class BoundingBox:
    """Represents a rectangular range of pixels.

    x0: Column of leftmost pixel included in range
    y0: Row of topmost pixel included in range
    w: Width of range [px]
    h: Height of range [px]
    """

    x0: int
    y0: int
    w: int
    h: int

    def x1(self):
        """Column of rightmost pixel included in range."""
        return self.x0 + self.w - 1

    def y1(self):
        """Row of bottommost pixel included in range."""
        return self.y0 + self.h - 1

    def clamped(self, other):
        """Compute the intersection of this bounding box with another."""
        x0 = max(self.x0, other.x0)
        y0 = max(self.y0, other.y0)
        x1 = max(min(self.x1(), other.x1()), x0 - 1)
        y1 = max(min(self.y1(), other.y1()), y0 - 1)
        return BoundingBox(x0, y0, x1 - x0 + 1, y1 - y0 + 1)

    def nonEmpty(self):
        """Return true iff there are pixels in this range."""
        return self.w > 0 and self.h > 0


@dataclass
class CameraRotation:
    """Represents the rotational motion of a camera about a fixed axis.

    u: Axis of rotation
    omega_dt: Angle of rotation during one row's readout time
    """

    u: np.array
    omega_dt: float


# s: Array of spherical coordinates
# row: Array of row numbers
# Returns: Array of rotated spherical coordinates
def rotate(rot, s, row):
    """
    Rotates the image based on the camera rotation
    """
    # TODO: Optimize for row is vector, not matrix
    th = -rot.omega_dt * row
    sth = np.sin(th)
    cth = np.cos(th)
    nr, nc, _ = s.shape
    u = rot.u
    # Construct rotation matrices manually to vectorize over pixels
    r = np.empty((nr, nc, 3, 3), dtype=np.float32)
    r[:, :, 0, 0] = cth + u[0] ** 2 * (1 - cth)
    r[:, :, 0, 1] = u[0] * u[1] * (1 - cth) - u[2] * sth
    r[:, :, 0, 2] = u[0] * u[2] * (1 - cth) + u[1] * sth
    r[:, :, 1, 0] = u[0] * u[1] * (1 - cth) + u[2] * sth
    r[:, :, 1, 1] = cth + u[1] ** 2 * (1 - cth)
    r[:, :, 1, 2] = u[1] * u[2] * (1 - cth) - u[0] * sth
    r[:, :, 2, 0] = u[0] * u[2] * (1 - cth) - u[1] * sth
    r[:, :, 2, 1] = u[1] * u[2] * (1 - cth) + u[0] * sth
    r[:, :, 2, 2] = cth + u[2] ** 2 * (1 - cth)
    return np.einsum('...ij,...j', r, s)


def tile_transform_bb(src, cam, rot, dst):
    """
    Reprojects the image from gnomonic to a stereographic projection
    """
    x0, y0 = cam.normalize_gn(src.x0, src.y0)
    x1, y1 = cam.normalize_gn(src.x1(), src.y1())

    s = gn_to_sph(np.array([x0, x1]), np.array([y0, y1]))
    rs = rotate(rot, s,
                np.array([[src.y0, src.y0],
                          [src.y1(), src.y1()]]))

    xst, yst = sph_to_st(rs)
    xstp, ystp = cam.st_to_px(xst, yst)
    xmin = floor(np.min(xstp))
    ymin = floor(np.min(ystp))
    bb = BoundingBox(xmin - dst.x0, ymin - dst.y0, ceil(np.max(xstp)) - xmin + 1, ceil(np.max(ystp)) - ymin + 1)

    # Coordinates relative to bounding box corner
    a = np.array([[0, 0, 1],
                  [0, src.h - 1, 1],
                  [src.w - 1, 0, 1],
                  [src.w - 1, src.h - 1, 1]], dtype=np.float32)
    b = np.array([[xstp[0, 0] - xmin, ystp[0, 0] - ymin],
                  [xstp[1, 0] - xmin, ystp[1, 0] - ymin],
                  [xstp[0, 1] - xmin, ystp[0, 1] - ymin],
                  [xstp[1, 1] - xmin, ystp[1, 1] - ymin]], dtype=np.float32)
    c = np.linalg.lstsq(a, b, rcond=None)[0].T
    return c, bb


def remap_roi(img, src, cam, rot):
    """
    Remaps the region of interest in 64x64 tiles to correct for rolling shutter
    """
    _, bb0 = tile_transform_bb(src, cam, rot, BoundingBox(0, 0, 0, 0))
    out = np.zeros((bb0.h, bb0.w, 3), dtype=np.uint8)
    dst = BoundingBox(0, 0, bb0.w, bb0.h)
    # Block size
    bs = 64
    for j in range(src.y0, src.y1() + 1, bs - 1):
        jb = min(j + bs, src.y1() + 1)
        for i in range(src.x0, src.x1() + 1, bs - 1):
            ib = min(i + bs, src.x1() + 1)
            bsrc = BoundingBox(i, j, ib - i, jb - j)
            c, bb = tile_transform_bb(bsrc, cam, rot, bb0)
            bbc = bb.clamped(dst)
            # Adjust translation of map for clamping
            c[0, 2] -= bbc.x0 - bb.x0
            c[1, 2] -= bbc.y0 - bb.y0

            # Perform remapping using only data from input tile (taking
            # advantage of the "transparent" border mode to avoid extrapolating
            # the map).
            cv2.warpAffine(img[j:jb, i:ib], c, (bbc.w, bbc.h),
                           dst=out[bbc.y0:(bbc.y1() + 1), bbc.x0:(bbc.x1() + 1)],
                           flags=cv2.INTER_CUBIC,
                           borderMode=cv2.BORDER_TRANSPARENT)
    return out, bb0


def bufferedRoi(x, y, w, h, wTot, hTot, b):
    """
    Buffers the region of interest by adding to the width and height of the box
    """
    xl = max(x - b, 0)
    xr = min(x + w + b, wTot)
    yl = max(y - b, 0)
    yr = min(y + h + b, hTot)
    return (xl, yl, xr - xl, yr - yl)

def drawContourCircle(img, xy, r, contours, showCircle):
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

def __findMinEnclosingCircle(img, highThresh, showCircle = False):
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
        if (showCircle):
            drawContourCircle(img, xy, r, contours, showCircle)
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
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 120,
                               param1=10, param2=20,
                               minRadius=0, maxRadius=int(maxRadius/2) + 1)
    if (circles is None):
        return __findMinEnclosingCircle(img, highThresh, showCircle)
    areas = [circleArea(circle) for circle in circles[0]]
    max_idx = np.argmax(areas)
    circle = circles[0][max_idx]
    xy = (int(circle[0]), int(circle[1]))
    r = int(circle[2])
    if(showCircle):
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
    if (body == 'e'):
        percentageThresh = EARTH_PERCENTAGE_THRESH
        highThreshRed = cv2.inRange(img, (0, 0, 5), (255, 255, 255))
        highThreshGreen = cv2.inRange(img, (0, 5, 0), (255, 255, 255))
        highThreshBlue = cv2.inRange(img, (5, 0, 0), (255, 255, 255))
        highThresh = highThreshRed + highThreshGreen + highThreshBlue
    elif (body == 's'):
        percentageThresh = SUN_PERCENTAGE_THRESH
        highThresh = cv2.inRange(img, (225, 225, 225), (255, 255, 255))
    else:
        percentageThresh = MOON_PERCENTAGE_THRESH
        highThresh = thresh
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

def find(src, camera_params:CameraParameters=CisLunarCameraParameters):
    cam = Camera(radians(camera_params.hFov), radians(camera_params.vFov), camera_params.hPix, camera_params.vPix)

    # u is in body frame here
    # Assumes only spinning about y-axis
    u = np.array([0, 1, 0], dtype=np.float32)
    camNum = int(re.search(r'[cam](\d+)', src).group(1))
    if camNum == 1:
        u = np.linalg.inv(camera_params.cam1Rotation).dot(u)
    elif camNum == 2:
        u = np.linalg.inv(camera_params.cam2Rotation).dot(u)
    elif camNum == 3:
        u = np.linalg.inv(camera_params.cam3Rotation).dot(u)
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
        logger.info("[OPNAV]: No contours found")
        return result, {}

    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    c = contours[max_index]
    # del contours[max_index]

    x, y, w, h = cv2.boundingRect(c)
    x, y, w, h = bufferedRoi(x, y, w, h, cam.w, cam.h, 16)
    box = BoundingBox(x, y, w, h)
    out, bbst = remap_roi(img, box, cam, rot)
    x = bbst.x0
    y = bbst.y0
    # TODO make sure we can handle eclipse edge case
    # Gets the next largest body that doesn't overlap with first body
    c2 = None
    x2, y2, w2, h2 = 0, 0, 0, 0
    out2 = None
    for con in contours:
        area = cv2.contourArea(con)
        if area >= 100:  # TODO: Placeholder
            x2, y2, w2, h2 = cv2.boundingRect(con)
            # Checks for rectangle overlap
            if (x + w < x2 or x > x2 + w2 or y < y2 + h2 or y2 + h2 > y):
                c2 = con
                x2, y2, w2, h2 = bufferedRoi(x2, y2, w2, h2, cam.w, cam.h, 16)
                box2 = BoundingBox(x2, y2, w2, h2)
                out2, bbst2 = remap_roi(img, box2, cam, rot)
                break
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
                body_values["Sun"] = [x + sX - 1640, y + sY - 1232, sR]
                sXst, sYst = cam.normalize_st(bbst.x0 + sX, bbst.y0 + sY)
                sRho2 = sXst ** 2 + sYst ** 2
                sDia = 4 * 2 * sR * (2 * cam.xmax_st / cam.w) / (4 + sRho2)
                sSx, sSy, sSz = st_to_sph(sXst, sYst)
                result.set_sun_detection(sSx, sSy, sSz, sDia)

    else:
        earth = None
        index = 0
        for f in [out, out2]:
            # if f is None or cv2.sumElems(f) == (0, 0, 0, 0) or measureSun(f, w, h) is not None:
            #     continue
            if earth is not None:
                earth = None
            elif np.max(areas) > 400 and index == 0: #TODO: Placeholder
                earth = measureEarth(f, w, h)
            if earth is not None:
                (eX, eY), eR = earth
                body_values["Earth"] = [x + eX - 1640, y + eY - 1232, eR]
                eXst, eYst = cam.normalize_st(bbst.x0 + eX, bbst.y0 + eY)
                eRho2 = eXst ** 2 + eYst ** 2
                eDia = 4 * 2 * eR * (2 * cam.xmax_st / cam.w) / (4 + eRho2)
                eSx, eSy, eSz = st_to_sph(eXst, eYst)
                result.set_earth_detection(eSx, eSy, eSz, eDia)
            elif earth is None:#TODO: Placeholder
                moon = measureMoon(f, w, h)
                if moon is not None:
                    (mX, mY), mR = moon
                    body_values["Moon"] = [x + mX - 1640, y + mY - 1232, mR]
                    mXst, mYst = None,None
                    # Checks whether moon contour is first or second contour
                    if index == 0:
                        mXst, mYst = cam.normalize_st(bbst.x0 + mX, bbst.y0 + mY)
                    if index == 1:
                        mXst, mYst = cam.normalize_st(bbst2.x0 + mX, bbst2.y0 + mY)
                    mRho2 = mXst ** 2 + mYst ** 2
                    mDia = 4 * 2 * mR * (2 * cam.xmax_st / cam.w) / (4 + mRho2)
                    mSx, mSy, mSz = st_to_sph(mXst, mYst)
                    result.set_moon_detection(mSx, mSy, mSz, mDia)
            index += 1
    return result, body_values

def findStereographic(src, camera_params:CameraParameters=CisLunarCameraParameters):
    cam = Camera(radians(camera_params.hFov), radians(camera_params.vFov), camera_params.hPix, camera_params.vPix)
    # u is in body frame here
    # Assumes only spinning about y-axis
    u = np.array([0, 1, 0], dtype=np.float32)
    camNum = int(re.search(r'[cam](\d+)', src).group(1))
    if camNum == 1:
        u = np.linalg.inv(camera_params.cam1Rotation).dot(u)
    elif camNum == 2:
        u = np.linalg.inv(camera_params.cam2Rotation).dot(u)
    elif camNum == 3:
        u = np.linalg.inv(camera_params.cam3Rotation).dot(u)
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
        logger.info("[OPNAV]: No contours found")
        return result, {}

    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    c = contours[max_index]
    # del contours[max_index]

    x, y, w, h = cv2.boundingRect(c)
    x, y, w, h = bufferedRoi(x, y, w, h, cam.w, cam.h, 16)
    bbst = BoundingBox(x, y, w, h)
    out = img[y:y + h, x:x + w]
    # TODO make sure we can handle eclipse edge case
    # Gets the next largest body that doesn't overlap with first body
    c2 = None
    x2, y2, w2, h2 = 0, 0, 0, 0
    out2 = None
    for con in contours:
        area = cv2.contourArea(con)
        if area >= 100:  # TODO: Placeholder
            x2, y2, w2, h2 = cv2.boundingRect(con)
            # Checks for rectangle overlap
            if (x + w < x2 or x > x2 + w2 or y < y2 + h2 or y2 + h2 > y):
                c2 = con
                x2, y2, w2, h2 = bufferedRoi(x2, y2, w2, h2, cam.w, cam.h, 16)
                box2 = BoundingBox(x2, y2, w2, h2)
                out2, bbst2 = remap_roi(img, box2, cam, rot)
                break
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
                body_values["Sun"] = [x + sX - 1640, y + sY - 1232, sR]
                sXst, sYst = cam.normalize_st(bbst.x0 + sX, bbst.y0 + sY)
                sRho2 = sXst ** 2 + sYst ** 2
                sDia = 4 * 2 * sR * (2 * cam.xmax_st / cam.w) / (4 + sRho2)
                sSx, sSy, sSz = st_to_sph(sXst, sYst)
                result.set_sun_detection(sSx, sSy, sSz, sDia)

    else:
        earth = None
        index = 0
        for f in [out, out2]:
            # if f is None or cv2.sumElems(f) == (0, 0, 0, 0) or measureSun(f, w, h) is not None:
            #     continue
            if earth is not None:
                earth = None
            elif np.max(areas) > 400 and index == 0: #TODO: Placeholder
                earth = measureEarth(f, w, h)
            if earth is not None:
                (eX, eY), eR = earth
                body_values["Earth"] = [x + eX - 1640, y + eY - 1232, eR]
                eXst, eYst = cam.normalize_st(bbst.x0 + eX, bbst.y0 + eY)
                eRho2 = eXst ** 2 + eYst ** 2
                eDia = 4 * 2 * eR * (2 * cam.xmax_st / cam.w) / (4 + eRho2)
                eSx, eSy, eSz = st_to_sph(eXst, eYst)
                result.set_earth_detection(eSx, eSy, eSz, eDia)
            elif earth is None:#TODO: Placeholder
                moon = measureMoon(f, w, h)
                if moon is not None:
                    (mX, mY), mR = moon
                    body_values["Moon"] = [x + mX - 1640, y + mY - 1232, mR]
                    mXst, mYst = None,None
                    # Checks whether moon contour is first or second contour
                    if index == 0:
                        mXst, mYst = cam.normalize_st(bbst.x0 + mX, bbst.y0 + mY)
                    if index == 1:
                        mXst, mYst = cam.normalize_st(bbst2.x0 + mX, bbst2.y0 + mY)
                    mRho2 = mXst ** 2 + mYst ** 2
                    mDia = 4 * 2 * mR * (2 * cam.xmax_st / cam.w) / (4 + mRho2)
                    mSx, mSy, mSz = st_to_sph(mXst, mYst)
                    result.set_moon_detection(mSx, mSy, mSz, mDia)
            index += 1
    return result, body_values

def test_center_finding(dir, results_file, st_gn = "st"):
    frames = []
    frames_dir = os.path.join(dir, "images/")
    for filename in os.listdir(frames_dir):
        if filename.endswith(st_gn + ".png"):
            frame = os.path.join(frames_dir, filename)
            frames.append(frame)
        else:
            continue
    o = open(os.path.join(dir, "observations.json"))
    c = open(os.path.join(dir, "cameras.json"))
    observations = json.load(o)
    cameras = json.load(c)
    st_scale = cameras["cameras"][0]["stereographic_scale"]
    all_truth_vals = {}
    for frame in observations["observations"][0]["frames"]:
        detections = frame["detections"]
        frame_truth_vals = {}
        for detection in detections:
            truthX = detection["center_st"][0] * st_scale
            truthY = detection["center_st"][1] * st_scale
            truthR = detection["radius_st"] * st_scale
            frame_truth_vals[detection["body"]] = [truthX, truthY, truthR]
        all_truth_vals[frame["image_stereographic"]] = frame_truth_vals
    results = []
    for i in range(len(frames)):
        perf_values = {}
        frame = frames[i]
        truths = all_truth_vals[frame.split('/')[-1]]
        _, body_vals = findStereographic(frame)
        sun_vals = body_vals.get("Sun")
        if (sun_vals):
            truth_vals = truths["Sun"]
            perf = []
            for j in range(3):
                perf += [sun_vals[j] - truth_vals[j]]
            perf_values["Sun"] = perf
            results.append(perf_values)
        earth_vals = body_vals.get("Earth")
        if (earth_vals):
            truth_vals = truths["Earth"]
            perf = []
            for j in range(3):
                perf += [earth_vals[j] - truth_vals[j]]
            perf_values["Earth"] = perf
            results.append(perf_values)
        moon_vals = body_vals.get("Moon")
        if (moon_vals):
            truth_vals = truths["Moon"]
            perf = []
            for j in range(3):
                perf += [moon_vals[j] - truth_vals[j]]
            perf_values["Moon"] = perf
            results.append(perf_values)
    with open(results_file, 'w', newline='') as csvfile:
        for result in results:
            writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            sun = result.get("Sun")
            earth = result.get("Earth")
            moon = result.get("Moon")
            if sun is not None:
                writer.writerow(["Sun"] + sun)
            if earth is not None:
                writer.writerow(["Earth"] + earth)
            if moon is not None:
                writer.writerow(["Moon"] + moon)
    return results

traj_case1c_sim_easy = "/Users/andrew/PycharmProjects/cislunarSimulation/traj-case1c_sim_easy/"
traj_case1c_sim_easy_results_file = "center_finding_results_traj-case1c_sim_easy.csv"
test_center_finding(traj_case1c_sim_easy, traj_case1c_sim_easy_results_file)

traj_case1c_sim = "/Users/andrew/PycharmProjects/cislunarSimulation/traj-case1c_sim/"
traj_case1c_sim_results_file = "center_finding_results_traj-case1c_sim.csv"
test_center_finding(traj_case1c_sim, traj_case1c_sim_results_file)
# findStereographic("/Users/andrew/PycharmProjects/cislunarSimulation/traj-case1c_sim_easy/images/cam3_expHigh_f0_dt10.47200_st.png")
# find("/Users/andrew/PycharmProjects/cislunarSimulation/traj-case1c_sim_easy/images/cam3_expHigh_f19_dt11.71555_st.png")

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
# * Bottleneck on RPi appears to be finding contours on full-res image.  Consider finding contours on low-res image, then scaling ROI
# if __name__ == "__main__":
#     """
#     Run "python3 find_with_contours.py -i=<IMAGE>" to test this module
#     """
#     ap = argparse.ArgumentParser()
#     ap.add_argument("-i", "--image", help="path to the image")
#     args = vars(ap.parse_args())
#     find(args["image"])

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
# * Bottleneck on RPi appears to be finding contours on full-res image.  Consider finding contours on low-res image, then scaling ROI

