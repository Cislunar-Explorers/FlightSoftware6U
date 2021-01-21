from dataclasses import dataclass
import cv2
import numpy as np
from math import pi, radians, tan, floor, ceil
import os
import copy

from core.find_with_kmeans import getkmeans

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
        self.xmax_st = 2*tan(hfov/4)
        self.ymax_st = 2*tan(vfov/4)
        # Maximum gnomonic x coordinate
        self.xmax_gn = tan(hfov/2)
        self.ymax_gn = tan(vfov/2)

    def normalize_st(self, xp, yp):
        """Convert pixel coordinates to stereographic coordinates."""
        x = self.xmax_st*(2*xp/(self.w - 1) - 1)
        y = self.ymax_st*(2*yp/(self.h - 1) - 1)
        return x, y

    def normalize_gn(self, xp, yp):
        """Convert pixel coordinates to gnomonic coordinates."""
        x = self.xmax_gn*(2*xp/(self.w - 1) - 1)
        y = self.ymax_gn*(2*yp/(self.h - 1) - 1)
        return x, y

    def st_to_px(self, x, y):
        """Convert stereographic coordinates to pixel coordinates."""
        xp = (x/self.xmax_st + 1)*(self.w - 1)/2
        yp = (y/self.ymax_st + 1)*(self.h - 1)/2
        return xp, yp

def gn_to_sph(xx, yy):
    """Convert gnomonic coordinates to spherical coordinates."""
    zm = np.reciprocal(np.sqrt(np.add.outer(yy**2 + 1, xx**2)))
    s = np.empty((len(yy), len(xx), 3), dtype=np.float32)
    s[:,:,0] = xx*zm
    s[:,:,1] = yy[np.newaxis].T*zm
    s[:,:,2] = -zm
    return s

def sph_to_st(s):
    """Convert spherical coordinates to stereographic coordinates"""
    x = 2*s[:,:,0]/(1 - s[:,:,2])
    y = 2*s[:,:,1]/(1 - s[:,:,2])
    return x, y

def st_to_sph(x, y):
    """Convert stereographic coordinates to spherical coordinates."""
    norm = x**2 + y**2 + 4
    return 4*x/norm, 4*y/norm, (norm - 8)/norm

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
    # TODO: Optimize for row is vector, not matrix
    th = -rot.omega_dt*row
    sth = np.sin(th)
    cth = np.cos(th)
    nr, nc, _ = s.shape
    u = rot.u
    # Construct rotation matrices manually to vectorize over pixels
    r = np.empty((nr, nc, 3, 3), dtype=np.float32)
    r[:,:,0,0] = cth + u[0]**2 * (1 - cth)
    r[:,:,0,1] = u[0]*u[1]*(1 - cth) - u[2]*sth
    r[:,:,0,2] = u[0]*u[2]*(1 - cth) + u[1]*sth
    r[:,:,1,0] = u[0]*u[1]*(1 - cth) + u[2]*sth
    r[:,:,1,1] = cth + u[1]**2 * (1 - cth)
    r[:,:,1,2] = u[1]*u[2]*(1 - cth) - u[0]*sth
    r[:,:,2,0] = u[0]*u[2]*(1 - cth) - u[1]*sth
    r[:,:,2,1] = u[1]*u[2]*(1 - cth) + u[0]*sth
    r[:,:,2,2] = cth + u[2]**2 * (1 - cth)
    return np.einsum('...ij,...j', r, s)

def tile_transform_bb(src, cam, rot, dst):
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
    bb = BoundingBox(xmin-dst.x0, ymin-dst.y0, ceil(np.max(xstp)) - xmin + 1, ceil(np.max(ystp)) - ymin + 1)

    # Coordinates relative to bounding box corner
    a = np.array([[0, 0, 1],
                  [0, src.h-1, 1],
                  [src.w-1, 0, 1],
                  [src.w-1, src.h-1, 1]], dtype=np.float32)
    b = np.array([[xstp[0,0]-xmin, ystp[0,0]-ymin],
                  [xstp[1,0]-xmin, ystp[1,0]-ymin],
                  [xstp[0,1]-xmin, ystp[0,1]-ymin],
                  [xstp[1,1]-xmin, ystp[1,1]-ymin]], dtype=np.float32)
    c = np.linalg.lstsq(a, b, rcond=None)[0].T
    return c, bb

def remap_roi(img, src, cam, rot):
    _, bb0 = tile_transform_bb(src, cam, rot, BoundingBox(0,0,0,0))
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
            c[0,2] -= bbc.x0 - bb.x0
            c[1,2] -= bbc.y0 - bb.y0

            # Perform remapping using only data from input tile (taking
            # advantage of the "transparent" border mode to avoid extrapolating
            # the map).
            cv2.warpAffine(img[j:jb,i:ib], c, (bbc.w,bbc.h),
                           dst=out[bbc.y0:(bbc.y1()+1),bbc.x0:(bbc.x1()+1)],
                           flags=cv2.INTER_CUBIC,
                           borderMode=cv2.BORDER_TRANSPARENT)

    return out, bb0

def bufferedRoi(x, y, w, h, wTot, hTot, b):
    xl = max(x-b, 0)
    xr = min(x+w+b, wTot)
    yl = max(y-b, 0)
    yr = min(y+h+b, hTot)
    return (xl, yl, xr-xl, yr-yl)

# Measure blue channel (least sensitive), assume full disk
def measureSun(img):
    thresh = 245
    bw = cv2.threshold(img[:,:,0], thresh, 255, cv2.THRESH_BINARY)[1]
    contours = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    #for c in contours:
    c = contours[0]
    xy, (major, minor), _ = cv2.fitEllipse(c)
    r = (major + minor)/4
    return xy, r

# Measure green channel (alternatively, value)
# TODO: Why only green channel?
def measureMoon(img):
    thresh = 128
    bw = cv2.threshold(img[:,:,1], thresh, 255, cv2.THRESH_BINARY)[1]
    contours = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    #for c in contours:
    c = contours[0]
    xy, (major, minor), ang = cv2.minAreaRect(c)
    r = major/2
    # TODO: Shift center based on aspect ratio, center of "mass"
    return xy, r


cam = Camera(radians(62.2), radians(48.8), 3280, 2464)
u = np.array([0, 1, 0], dtype=np.float32)
#omega = 78/60 * 2 * pi
omega = 0
dt = 18.904e-6
rot = CameraRotation(u, -omega*dt)

#img = cv2.imread('sun1.jpg')
img = cv2.imread(os.path.join("D:", "OpNav", "data", "EclipseAndCrescentImages", "images", "EarthMoon.jpg"))
imgCopy = img.copy()

# In-place blur to reduce noise, avoid hot pixels
img = cv2.GaussianBlur(img, (5,5), 0, dst=img)

# Extract and threshold blue channel (least sensitive)
thresh = 64 # 245
bw = cv2.threshold(img[:,:,0], thresh, 255, cv2.THRESH_BINARY)[1]
cv2.imshow("Blue channel threshold", bw)

# bw = getkmeans(img)
# thresh = 10 # 245
# bw = cv2.threshold(img[:,:,0], thresh, 255, cv2.THRESH_BINARY)[1]
# bw = cv2.medianBlur(bw, 51)
# cv2.imshow("Segmented", bw)
# cv2.waitKey(0)

contours = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Hack around API breakage between OpenCV versions
contours = contours[0] if len(contours) == 2 else contours[1]

cv2.drawContours(imgCopy, contours, -1, (0,255,0), 3)
cv2.imshow("Contours", imgCopy)

#for c in contours:
c = contours[0]
x, y, w, h = cv2.boundingRect(c)
x, y, w, h = bufferedRoi(x, y, w, h, cam.w, cam.h, 16)
src = BoundingBox(x, y, w, h)

#src = BoundingBox(0, 0, cam.w, cam.h)
out, bbst = remap_roi(img, src, cam, rot)

# Measure body in region-of-interest
#(x, y), r = measureSun(out)
(x, y), r = measureMoon(out)

# Shift stereographic coordinates of center to camera frame (at start of exposure)
xst, yst = cam.normalize_st(bbst.x0 + x, bbst.y0 + y)

# Adjust diameter for stereographic distortion
rho2 = xst**2 + yst**2
dia = 4*2*r*(2*cam.xmax_st/cam.w)/(4 + rho2)

# Compute spherical coordinates of center (in camera frame at start of exposure)
sx, sy, sz = st_to_sph(xst, yst)

# Location of circle center in spherical coordinates,
# Angular diameter of circle in radians
print(sx, sy, sz, dia)

cv2.waitKey(0)

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