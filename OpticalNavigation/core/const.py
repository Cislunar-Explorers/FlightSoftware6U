import os
import numpy as np

# The system will wait for the expected time elapsed 
# for the spacecraft to face the angle that is 45 degrees
# away the current angle.
ACQUISITION_ANGLE_INCREMENT = 45 # degrees

# The amount the spacecraft must rotate in order to reach
# the next target angle from which the photo will be taken.
# This is an initial estimate and can be incremented by the 
# compensation amount if angular velocity is very fast.
ACQUISITION_ANGLE_DISPLACEMENT = 315 # degrees

# If wait time is too small (negative), then extra 
# rotations are added to ensure the system can 
# appropriately prepare itself to take the next photo.
ACQUISITION_COMPENSATION_ROTATION = 360 # degrees

# When OpNav is triggered, the direction the spacecraft is
# facing in is assumed to be 0 degrees. To ensure correct
# timing, acquisition waits for the spacecraft to rotate
# 315 degrees from start to take the first photo.
ACQUISITION_START_ANGLE = 315 # degrees

class CisLunarCameraParameters:
    # Camera constants
    # Horizontal/Vertical Field of View (Degrees), Pixel Dimensions
    hFov = 62.2
    vFov = 48.8
    hPix = 1685
    vPix = 813
    #Angular Separation Between Cameras (degrees)
    dcam12 = 60
    dcam13 = -60
    dcam23 = -120

class CameraAcquisionDirectoryNotFound(Exception):
    def __init__(self, camLoc):
        self.camLoc = camLoc
    def __str__(self):
        return '\"{}\" is not a valid camera acquisition directory'.format(self.camLoc)

class NoImagesInCameraAcquisitionDirectory(Exception):
    def __init__(self, camLoc):
        self.camLoc = camLoc
    def __str__(self):
        return 'No images found in camera acquisition directory \"{}\"'.format(self.camLoc)

class InvalidBodyNameForLoadProperties(Exception):
    def __init__(self, name):
        self.name = name
    def __str__(self):
        return '\"{}\" should be one of (\"{}\",\"{}\",\"{}\"). Was: \"{}\"'.format("name", "earth", "moon", "sun", self.name)

# Attitude

# Control local error quaternion vector of generalized Rodrigues parameters
_a=1
_f=2.*(_a+1)

# Tuning Parameters for Sigma Points
NX = 6.                                 # number of states
# TODO: These parameters are not used
#ALPHA = 0                           # determines spread of sigma points
#BETA = 2.                               # optimal for Gaussian distribution
#KAPPA = -3.                             # chosen such that KAPPA+NX=3
LAM = 0#ALPHA**2. * (KAPPA + NX) - NX     # depends on other variables
P0 = np.array([[1.e-1, 0., 0., 0., 0., 0.],
                  [0., 1.e-1, 0., 0., 0., 0.],
                  [0., 0., 1.e-1, 0., 0., 0.],
                  [0., 0., 0., 9.7e-10, 0., 0.],
                  [0., 0., 0., 0., 9.7e-10, 0.],
                  [0., 0., 0., 0., 0., 9.7e-10]]) * 10.

class TrajUKFConstants:
    # How wrong our dynamics model is? e.g. how off in variance will we be due
    # to solar radiation pressure, galactic particles, and bad gravity model? 
    # Units: (km^2)
    Q = np.diag(np.array([1, 1, 1, 1e-5, 1e-6, 1e-5], dtype=np.float))
    Sv = np.linalg.cholesky(Q)
    # How bad are our sensors? 
    # Units: (pixels^2), 
    PIXEL_ERROR = 1
    R = np.diag(np.array([1, 1, 1, 1, 1, 1], dtype=np.float)) * PIXEL_ERROR
    alpha = 10e-4
    beta = 2

    ue = 3.986e14
    um = 4.904e12
    us = 1.327e20
    re = 6378 * 1000  
    rm = 1737 * 1000
    rs = 695505 * 1000
