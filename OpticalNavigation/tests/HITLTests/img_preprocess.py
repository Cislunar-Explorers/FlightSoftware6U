import cv2
import numpy as np
from math import radians, tan, pi, sin, cos
#from scipy.spatial.transform import Rotation


def preprocess(img_src, img_dest):
    # Performs image transformation and rolling shutter corrections

    # img = cv2.imread('../rpispheres.png')
    img = cv2.imread(img_src)
    (nr, nc, _) = img.shape
    print('%d x %d' % (nc, nr))

    xmap = np.empty((nr, nc), dtype=np.float32)
    ymap = np.empty((nr, nc), dtype=np.float32)

    # Camera and coordinate system parameters
    hfov = radians(62.2)  # Horizontal field-of-view [rad]
    vfov = radians(48.8)  # Vertical field-of-view [rad]
    xmax_d = 2 * tan(hfov / 4)  # Maximum rectilinear x coordinate
    ymax_d = 2 * tan(vfov / 4)
    xmax_s = tan(hfov / 2)  # Maximum stereographic x coordinate
    ymax_s = tan(vfov / 2)

    # Rolling shutter parameters
    # Rotation rate [rad/s]
    # omega = 0
    omega = 78 / 60 * 2 * pi
    # dt = 18.904e-6
    dt = 18.904e-6 * 4  # Downsampled

    # Generate normalized coordinates of output image
    xx = np.linspace(-xmax_d, xmax_d, nc, dtype=np.float32)
    yy = np.linspace(-ymax_d, ymax_d, nr, dtype=np.float32)

    # Project output pixels to sphere using inverse stereographic projection
    norm = np.add.outer(4 + yy ** 2, xx ** 2)
    s = np.empty((nr, nc, 3), dtype=np.float32)
    s[:, :, 0] = 4 * xx / norm
    s[:, :, 1] = 4 * yy[np.newaxis].T / norm
    s[:, :, 2] = (norm - 8) / norm

    # Rotate according to output row number
    u = np.array([0, 1, 0], dtype=np.float32)

    # Initial guess
    row = np.tile(np.linspace(0, nr - 1, nr, dtype=np.float32)[np.newaxis].T, nc)

    print("B4 rotate")

    # TODO: preallocate?
    def rotate(u, th, s):
        th = -omega * dt * row
        sth = np.sin(th)
        cth = np.cos(th)
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

    rs = rotate(u, th, s)
    # rs[rs[:,:,2] > -0.1, 2] = -0.1
    ymap = -rs[:, :, 1] / rs[:, :, 2] * (nr - 1) / (2 * ymax_s) + (nr - 1) / 2

    xmap = -rs[:, :, 0] / rs[:, :, 2] * (nc - 1) / (2 * xmax_s) + (nc - 1) / 2
    mask = (ymap >= 0) & (ymap <= nr) & (xmap >= 0) & (xmap <= nc) & (rs[:, :, 2] < -0.1)

    # Solve for which row each mapped pixel comes from (uses fixed-point iteration)
    print('%s %s' % (np.max(mask * np.abs(ymap - row)), np.median(np.abs(ymap - row))))
    while np.max(mask * np.abs(ymap - row)) > 0.5:
        # TODO: Implement a better technique than fixed-point iteration
        # Clip needed to avoid fractal divergence basins
        row = np.clip(ymap, -1, nr + 1)
        th = -omega * dt * row
        rs = rotate(u, th, s)
        ymap = -rs[:, :, 1] / rs[:, :, 2] * (nr - 1) / (2 * ymax_s) + (nr - 1) / 2

        xmap = -rs[:, :, 0] / rs[:, :, 2] * (nc - 1) / (2 * xmax_s) + (nc - 1) / 2
        mask = (ymap >= 0) & (ymap <= nr) & (xmap >= 0) & (xmap <= nc) & (rs[:, :, 2] < -0.1)

        print('%s %s' % (np.max(mask * np.abs(ymap - row)), np.median(np.abs(ymap - row))))

    # Spherical to gnomonic pixels
    xmap = -rs[:, :, 0] / rs[:, :, 2] * (nc - 1) / (2 * xmax_s) + (nc - 1) / 2
    ymap = -rs[:, :, 1] / rs[:, :, 2] * (nr - 1) / (2 * ymax_s) + (nr - 1) / 2

    dst = cv2.remap(img, xmap, ymap, cv2.INTER_LANCZOS4)
    cv2.imwrite(img_dest, dst)

