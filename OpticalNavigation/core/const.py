import os
import numpy

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

# Position Dynamics
MAIN_THRUST_ACCELERATION = 9.81 # m/s^2

# ATTITUDE
SPACECRAFT_MASS = 10   #spacecraft mass, kg
SPACECRAFT_HEIGHT = .3   #spacecraft height, meters
SPACECRAFT_WIDTH = .3   #spacecraft width, meters
SPACECRAFT_DEPTH = .1   #spacecraft depth, meters
SPACECRAFT_I_B = numpy.array([[(1./12.)*SPACECRAFT_MASS*(SPACECRAFT_HEIGHT**2. + SPACECRAFT_DEPTH**2.), 0., 0.],
                  [0., (1./12.)*SPACECRAFT_MASS*(SPACECRAFT_WIDTH**2. + SPACECRAFT_DEPTH**2.), 0.],
                  [0., 0., (1./12.)*SPACECRAFT_MASS*(SPACECRAFT_WIDTH**2. + SPACECRAFT_HEIGHT**2.)]]) #spacecraft inertia tensor

DAMPER_MASS = 8 #damper mass in kg
DAMPER_RADIUS = 0.1 #damper radius in meters
DAMPER_C = 0.9    #damping coefficient
DAMPER_I_D = numpy.array([[(2./5.)*DAMPER_MASS*DAMPER_RADIUS**2., 0., 0.],
                  [0., (2./5.)*DAMPER_MASS*DAMPER_RADIUS**2., 0.],
                  [0., 0., (2./5.)*DAMPER_MASS*DAMPER_RADIUS**2.]])

# _q0 = [0., 0., 0., 1.]                   # initial quaternion
# omega_init = [0., 0.001, 2., 0., 0., 0.]# initial angular velocity
TOTAL_INTEGRATION_TIME = 700                             # total integration time
INTEGRATION_TIMESTEP = 0.001                          # integration timestep
GYRO_SAMPLE_RATE = 1

# TODO: Stochastic accelerations not used in attitude UKF
# _sigma_acc = 0.                          # stochastic accelerations
# The following values will need to be calculated for our gyro. Values will differ significantly which will affect the output.
GYRO_SIGMA = 1.e-10
GYRO_NOISE_SIGMA = 1.e-7
BIAS_INIT=[0., 0., 0.]
MEAS_SIGMA = 8.7e-4

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
P0 = numpy.array([[1.e-1, 0., 0., 0., 0., 0.],
                  [0., 1.e-1, 0., 0., 0., 0.],
                  [0., 0., 1.e-1, 0., 0., 0.],
                  [0., 0., 0., 9.7e-10, 0., 0.],
                  [0., 0., 0., 0., 9.7e-10, 0.],
                  [0., 0., 0., 0., 0., 9.7e-10]]) * 10.

Q = numpy.array([[GYRO_NOISE_SIGMA**2. - (1./6.)*GYRO_SIGMA**2.*GYRO_SAMPLE_RATE**2., 0., 0., 0., 0., 0.],
                [0., GYRO_NOISE_SIGMA**2. - (1./6.)*GYRO_SIGMA**2.*GYRO_SAMPLE_RATE**2., 0., 0., 0., 0.],
                [0., 0., GYRO_NOISE_SIGMA**2. - (1./6.)*GYRO_SIGMA**2.*GYRO_SAMPLE_RATE**2., 0., 0., 0.],
                [0., 0., 0., GYRO_SIGMA**2., 0., 0.],
                [0., 0., 0., 0., GYRO_SIGMA**2., 0.],
                [0., 0., 0., 0., 0., GYRO_SIGMA**2.]]) * .5*GYRO_SAMPLE_RATE

R = numpy.eye(9) * MEAS_SIGMA**2.
