import utils.parameters as params

import os
import numpy as np
from enum import Enum
import math

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

class CameraParameters:
    """
    Contains camera specifications
    """
    def __init__(self, hFov:np.float, vFov:np.float, hPix:np.int, vPix:np.int, 
                angular_separation_cam1_cam2:np.float, angular_separation_cam1_cam3:np.float, 
                angular_separation_cam2_cam3:np.float, c1az:np.float, c1ay:np.float, c2az:np.float, c2ay:np.float,
                 c3az:np.float) -> None:
        # Camera constants
        # Horizontal/Vertical Field of View (Degrees), Pixel Dimensions
        self.hFov = hFov
        self.vFov = vFov
        self.hPix = hPix
        self.vPix = vPix
        #Angular Separation Between Cameras (degrees)
        self.dcam12 = angular_separation_cam1_cam2
        self.dcam13 = -angular_separation_cam1_cam3
        self.dcam23 = -angular_separation_cam2_cam3

        c1az = math.radians(c1az)
        c1ay = math.radians(c1ay)
        c2az = math.radians(c2az)
        c2ay = math.radians(c2ay)
        c3az = math.radians(c3az)

        c1rz = np.array([math.cos(c1az), -1 * math.sin(c1az), 0, math.sin(c1az), math.cos(c1az), 0, 0, 0, 1]).reshape(3,3)
        c1ry = np.array([math.cos(c1ay), 0, math.sin(c1ay), 0, 1, 0, -1 * math.sin(c1ay), 0, math.cos(c1ay)]).reshape(3,3)

        c2rz = np.array([math.cos(c2az), -1 * math.sin(c2az), 0, math.sin(c2az), math.cos(c2az), 0, 0, 0, 1]).reshape(3,3)
        c2ry = np.array([math.cos(c2ay), 0, math.sin(c2ay), 0, 1, 0, -1 * math.sin(c2ay), 0, math.cos(c2ay)]).reshape(3, 3)

        c3rz = np.array([math.cos(c3az), -1 * math.sin(c3az), 0, math.sin(c3az), math.cos(c3az), 0, 0, 0, 1]).reshape(3,3)

        self.cam1Rotation = np.matmul(c1rz, c1ry)
        self.cam2Rotation = np.matmul(c2rz, c2ry)
        self.cam3Rotation = c3rz

CisLunarCameraParameters = CameraParameters(62.2, 48.8, 1685, 813, 60, -60, -120, 90, 60, 90, -60, 53)

class CameraRecordingParameters:
    """
    Contains camera specifications for recording
    """
    def __init__(self, fps, recTime, expLow, expHigh ) -> None:
        self.fps = fps
        self.recTime = recTime
        self.expLow = expLow
        self.expHigh = expHigh


CisLunarCamRecParams = CameraRecordingParameters(params.CAMERA_FPS, params.CAMERA_RECORDING_TIME, params.CAMERA_LOW_EXPOSURE, params.CAMERA_HIGH_EXPOSURE)

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

# Opnav Constants
class OPNAV_EXIT_STATUS(Enum):
    SUCCESS = 0
    FAILURE = 1
    NO_TRAJECTORY_ENTRY_FOUND = 2
    NO_ATTITUDE_ENTRY_FOUND = 3
    NO_CAMMEAS_ENTRY_FOUND = 4
    ONE_OR_LESS_GYROMEAS_ENTRIES_FOUND = 5
    NO_EPHEMERIS_ENTRY_FOUND = 6

class ImageDetectionCircles:
    """
    Stores circle center of detected Sun, Moon and Earth in spherical coordinates
    as well as the angular diameter. 
    """
    def __init__(self) -> None:
        self.__earth_detection = None
        self.__moon_detection = None
        self.__sun_detection = None
    
    def set_earth_detection(self, spherical_x:np.float, spherical_y:np.float, spherical_z:np.float, angular_diameter:np.float) -> None:
        self.__earth_detection = np.array([spherical_x, spherical_y, spherical_z, angular_diameter], dtype=np.float)

    def set_moon_detection(self, spherical_x:np.float, spherical_y:np.float, spherical_z:np.float, angular_diameter:np.float) -> None:
        self.__moon_detection = np.array([spherical_x, spherical_y, spherical_z, angular_diameter], dtype=np.float)

    def set_sun_detection(self, spherical_x:np.float, spherical_y:np.float, spherical_z:np.float, angular_diameter:np.float) -> None:
        self.__sun_detection = np.array([spherical_x, spherical_y, spherical_z, angular_diameter], dtype=np.float)

    def get_earth_detection(self) -> np.ndarray:
        return self.__earth_detection

    def get_moon_detection(self) -> np.ndarray:
        return self.__moon_detection

    def get_sun_detection(self) -> np.ndarray:
        return self.__sun_detection

class Vector3:
    def __init__(self, x:np.float, y:np.float, z:np.float):
        self.data = np.array([x,y,z], dtype=np.float)

class Vector6:
    def __init__(self, x1:np.float, x2:np.float, x3:np.float, x4:np.float, x5:np.float, x6:np.float):
        self.data = np.array([x1, x2, x3, x4, x5, x6], dtype=np.float)

class Vector4:
    def __init__(self, x1:np.float, x2:np.float, x3:np.float, x4:np.float):
        self.data = np.array([x1, x2, x3, x4], dtype=np.float)

"""
Measurement vectors
"""
class CameraMeasurementVector(Vector6):
    """
    Stores camera measurement quantities into 6D array [z1, z2, z3, z4, z5, z6]
    [ang_em]: (z1) angular separation between the Earth and Moon in radians
    [ang_es]: (z2) angular separation between the Earth and Sun in radians
    [ang_ms]: (z3) angular separation between the Moon and Sun in radians
    [e_dia]: (z4) angular diameter of the Earth (radians)
    [m_dia]: (z5) angular diameter of the Moon (radians)
    [s_dia]: (z6) angular diameter of the Sun (radians)
    """
    def __init__(self, ang_em:np.float, ang_es:np.float, ang_ms:np.float, e_dia:np.float, m_dia:np.float, s_dia:np.float):
        super().__init__(x1=ang_em, x2=ang_es, x3=ang_ms, x4=e_dia, x5=m_dia, x6=s_dia)

    def get_angular_separation_earth_moon(self) -> np.float:
        return self.data[0]

    def get_angular_separation_earth_sun(self) -> np.float:
        return self.data[1]

    def get_angular_separation_moon_sun(self) -> np.float:
        return self.data[2]

    def get_angular_diameter_earth(self) -> np.float:
        return self.data[3]

    def get_angular_diameter_moon(self) -> np.float:
        return self.data[4]

    def get_angular_diameter_sun(self) -> np.float:
        return self.data[5]

class GyroMeasurementVector(Vector3):
    """
    Contains angular velocity readings from the gyroscope which are stored in a 3d numpy array.
    TODO: The units are currently undecided.
    """
    def __init__(self, omega_x: np.float, omega_y: np.float, omega_z: np.float):
        super().__init__(x=omega_x, y=omega_y, z=omega_z)

class QuaternionVector(Vector4):
    """
    Stores quaternion quantities in a 4D numpy array [q1, q2, q3, q4] of form
    (q1)i + (q2)j + (q3)k + q4
    """
    def __init__(self, q1: np.float, q2: np.float, q3: np.float, q4: np.float):
        super().__init__(x1=q1, x2=q2, x3=q3, x4=q4)

    @classmethod
    def from_numpy_array(cls, quat: np.ndarray):
        """
        Creates an QuaternionVector object from [quat] numpy array.
        @precondition
        [quat] must be a 4D matrix
        """
        quat = quat.flatten()
        assert quat.shape[0] == 4
        return cls(q1=quat[0], q2=quat[1], q3=quat[2], q4=quat[3])

    def get_q1(self):
        return self.data[0]

    def get_q2(self):
        return self.data[1]

    def get_q3(self):
        return self.data[2]

    def get_q4(self):
        return self.data[3]

class AttitudeStateVector(Vector6):
    """
    Stores attitude state vector in a 6D numpy array [rod_param1, rod_param2, rod_param3, bias1, bias2, bias3] where
    the first three parameters are attitude errors represented by Rodriguez Parameters and the last three are 
    gyroscope biases.
    """
    def __init__(self, rod_param1: np.float, rod_param2: np.float, rod_param3: np.float, bias1: np.float, bias2: np.float, bias3: np.float):
        super().__init__(x1=rod_param1, x2=rod_param2, x3=rod_param3, x4=bias1, x5=bias2, x6=bias3)

    @classmethod
    def from_numpy_array(cls, state: np.ndarray):
        """
        Creates an AttitudeStateVector object from [state] numpy array.
        @precondition
        [state] must be a 6D matrix
        """
        state = state.flatten()
        assert state.shape[0] == 6
        return cls(rod_param1=state[0], rod_param2=state[1], rod_param3=state[2], bias1=state[3], bias2=state[4], bias3=state[5])

    def get_rod_params(self):
        return self.data.reshape(6,)[0:3]
    
    def get_biases(self):
        return self.data.reshape(6,)[3:6]

class EphemerisVector(Vector6):
    """
    Stores ephemeris of a body in a 6D numpy array [x pos (km), y pos (km), z pos (km), x vel (km/s), y vel (km/s), z vel (km/s)]
    Coordinates are assumed to be in J2000 ECI frame.
    """
    def __init__(self, x_pos: np.float, y_pos: np.float, z_pos: np.float, x_vel: np.float, y_vel: np.float, z_vel: np.float):
        super().__init__(x1=x_pos, x2=y_pos, x3=z_pos, x4=x_vel, x5=y_vel, x6=z_vel)

    def get_position(self):
        return self.data.reshape(6,)[0:3]

    def get_velocity(self):
        return self.data.reshape(6,)[3:6]

class TrajectoryStateVector(Vector6):
    """
    Stores satellite trajectory state in a 6D numpy array [x pos (km), y pos (km), z pos (km), x vel (km/s), y vel (km/s), z vel (km/s)]
    Coordinates are assumed to be in J2000 ECI frame.
    """
    def __init__(self, x_pos: np.float, y_pos: np.float, z_pos: np.float, x_vel: np.float, y_vel: np.float, z_vel: np.float):
        super().__init__(x1=x_pos, x2=y_pos, x3=z_pos, x4=x_vel, x5=y_vel, x6=z_vel)

    @classmethod
    def from_numpy_array(cls, state: np.ndarray):
        """
        Creates a TrajectoryStateVector object from [state] numpy array.
        @precondition
        [state] must be a 6D matrix
        """
        state = state.flatten()
        assert state.shape[0] == 6
        return cls(x_pos=state[0], y_pos=state[1], z_pos=state[2], x_vel=state[3], y_vel=state[4], z_vel=state[5])

    def get_position_data(self):
        return self.data.reshape(6,)[0:3]

    def get_velocity_data(self):
        return self.data.reshape(6,)[3:6]

class Matrix6x6:
    """
    Contains 6x6 matrix, stores in a numpy matrix
    """
    
    def __init__(self, r1:Vector6, r2:Vector6, r3:Vector6, r4:Vector6, r5:Vector6, r6:Vector6) -> None:
        """
        Matrix6x6 Constructor
        [r1]: Vector6 row 1
        [r2]: Vector6 row 2
        [r3]: Vector6 row 3
        [r4]: Vector6 row 4
        [r5]: Vector6 row 5
        [r6]: Vector6 row 6
        """
        self.data = np.array([r1.data.reshape(6,), r2.data.reshape(6,), r3.data.reshape(6,), r4.data.reshape(6,), r5.data.reshape(6,), r6.data.reshape(6,)], dtype=np.float)
        self.data = self.data.reshape(6,6)

    def __init__(self, matrix:np.ndarray) -> None:
        assert matrix.shape[0] == matrix.shape[1] == 6
        self.data = matrix

class CovarianceMatrix(Matrix6x6):
    """
    Contains covariance matrix for trajectory and attitude UKFs.
    """

    def get_confidence(self) -> np.float:
        """
        Get confidence of covariance matrix.
        """
        return np.trace(self.data)

"""
UKF Constants
"""
class TrajUKFConstants:
    # How wrong our dynamics model is? e.g. how off in variance will we be due
    # to solar radiation pressure, galactic particles, and bad gravity model? 
    # Units: (km^2)
    P0 = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
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
    ACCELERATION_DIR = QuaternionVector(q1=1, q2=0, q3=0, q4=0)

class AttitudeUKFConstants:
    # Control local error quaternion vector of generalized Rodrigues parameters
    _a=1
    _f=2.*(_a+1)

    # Tuning Parameters for Sigma Points
    NX = 6.                                 # number of states
    # TODO: These parameters are not used
    ALPHA = 0                           # determines spread of sigma points
    BETA = 2.                               # optimal for Gaussian distribution
    KAPPA = -3.                             # chosen such that KAPPA+NX=3
    LAM = 0#ALPHA**2. * (KAPPA + NX) - NX     # depends on other variables
    P0 = np.array([[1.e-1, 0., 0., 0., 0., 0.],
                    [0., 1.e-1, 0., 0., 0., 0.],
                    [0., 0., 1.e-1, 0., 0., 0.],
                    [0., 0., 0., 9.7e-10, 0., 0.],
                    [0., 0., 0., 0., 9.7e-10, 0.],
                    [0., 0., 0., 0., 0., 9.7e-10]]) * 10.
    default_gyro_sigma = 1.e-10
    default_gyro_sample_rate = 0.01
    default_gyro_noise_sigma = 1.e-7
    default_meas_sigma = 8.7e-4

class GyroVars:
    def __init__(self):
        self.gyro_sigma=AttitudeUKFConstants.default_gyro_sigma
        self.gyro_sample_rate=AttitudeUKFConstants.default_gyro_sample_rate
        self.gyro_noise_sigma=AttitudeUKFConstants.default_gyro_noise_sigma
        self.meas_sigma=AttitudeUKFConstants.default_meas_sigma

    def get_Q_matrix(self):
        return np.array([[self.gyro_noise_sigma**2. - (1./6.)*self.gyro_sigma**2.*self.gyro_sample_rate**2., 0., 0., 0., 0., 0.],
                    [0., self.gyro_noise_sigma**2. - (1./6.)*self.gyro_sigma**2.*self.gyro_sample_rate**2., 0., 0., 0., 0.],
                    [0., 0., self.gyro_noise_sigma**2. - (1./6.)*self.gyro_sigma**2.*self.gyro_sample_rate**2., 0., 0., 0.],
                    [0., 0., 0., self.gyro_sigma**2., 0., 0.],
                    [0., 0., 0., 0., self.gyro_sigma**2., 0.],
                    [0., 0., 0., 0., 0., self.gyro_sigma**2.]]) * .5*self.gyro_sample_rate

    def get_R_matrix(self):
        return np.eye(9) * self.meas_sigma**2.

"""
Main Thrust Info
"""
class MainThrustInfo:
    """
    Thrust fire event information for trajectory UKF processing.
    """
    def __init__(self, kick_orientation:QuaternionVector, acceleration_magnitude:np.float) -> None:
        self.__kick_orientation = kick_orientation
        self.__kick_orientation.data = self.__kick_orientation.data / np.linalg.norm(self.__kick_orientation.data)
        self.__acceleration_magnitude = acceleration_magnitude
    
    def get_kick_orientation(self) -> QuaternionVector:
        return self.__kick_orientation

    def get_acceleration_magnitude(self) -> np.float:
        return self.__acceleration_magnitude

"""
UKF outputs
"""
class TrajectoryEstimateOutput:
    """
    Contains output quantities for trajectory UKF estimate: new state, covariance matrix and Kalman Gain.
    """
    def __init__(self, new_state:TrajectoryStateVector, new_P:CovarianceMatrix, K:Matrix6x6) -> None:
        self.new_state = new_state
        self.new_P = new_P
        self.K = K

class AttitudeEstimateOutput:
    """
    Contains output quantities for attitude UKF estimate: new state, covariance matrix and new quaternion.
    """
    def __init__(self, new_state:AttitudeStateVector, new_P:CovarianceMatrix, new_quat:QuaternionVector) -> None:
        self.new_state = new_state
        self.new_P = new_P
        self.new_quat = new_quat
