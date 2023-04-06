from fsw.opnav.core.const import CameraParameters
import os
import numpy as np

# class MatlabTestCameraParameters:
#     # Camera constants
#     # Horizontal/Vertical Field of View (Degrees), Pixel Dimensions
#     hFov = 62.2
#     vFov = 48.8
#     hPix = 3280
#     vPix = 813
#     #Angular Separation Between Cameras (degrees)
#     dcam12 = 60
#     dcam13 = -60
#     dcam23 = -120

MatlabTestCameraParameters = CameraParameters(
    62.2, 48.8, 3280, 813, 60, -60, -120, 90, 60, 90, -60, 53
)

CesiumTestCameraParameters = CameraParameters(
    62.2, 48.8, 640, 480, 60, -60, -120, 90, 60, 90, -60, 53
)  # Cam 2 looks down, Cam 3 looks up

TEST_DATA_DIR = os.path.join(os.path.dirname(__file__), "data")

TEST_ECLIPSEANDCRESCENTIMAGES = os.path.join(TEST_DATA_DIR, "EclipseAndCrescentImages")
TEST_FIND_DATASET_IMAGE_DIR = "images"
TEST_FIND_DATASET_CIRCLES_DIR = os.path.join("circles", "circles.csv")

# TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec =
#   os.path.join(TEST_DATA_DIR,'EM1_3DOF_Trajectory_June_27_2020_3600sec')
# TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations = 'iterations'
# TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory = os.path.join('trajectory','trajectory.csv')
# TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph = os.path.join('ephemeris','moon_eph.csv')
# TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph = os.path.join('ephemeris','sun_eph.csv')

# CISLUNAR_TEST_TRAJ = os.path.join(TEST_DATA_DIR, 'CislunarFullTraj_60secs')
# CISLUNAR_TEST_TRAJ_moonEph = os.path.join(CISLUNAR_TEST_TRAJ, 'ephemeris', 'sampled_moon_eph.csv')
# CISLUNAR_TEST_TRAJ_sunEph = os.path.join(CISLUNAR_TEST_TRAJ, 'ephemeris', 'sampled_sun_eph.csv')
# CISLUNAR_TEST_TRAJ_traj = os.path.join(CISLUNAR_TEST_TRAJ, 'trajectory', 'trajectory.csv')
# CISLUNAR_TEST_TRAJ_att = os.path.join(CISLUNAR_TEST_TRAJ, 'attitude', 'attitude.csv')

TEST_C1_DISCRETIZED = os.path.join(TEST_DATA_DIR, "c1_discretized")
TEST_C1_DISCRETIZED_meas = os.path.join(
    TEST_C1_DISCRETIZED, "measurements", "measurements.csv"
)
TEST_C1_DISCRETIZED_moonEph = os.path.join(
    TEST_C1_DISCRETIZED, "ephemeris", "moon_eph.csv"
)
TEST_C1_DISCRETIZED_sunEph = os.path.join(
    TEST_C1_DISCRETIZED, "ephemeris", "sun_eph.csv"
)
TEST_C1_DISCRETIZED_traj = os.path.join(
    TEST_C1_DISCRETIZED, "trajectory", "trajectory.csv"
)
TEST_C1_DISCRETIZED_matlab = os.path.join(
    TEST_C1_DISCRETIZED, "c1_discretized_ukf_k0.csv"
)

TEST_6HOURS = os.path.join(TEST_DATA_DIR, "6Hours")
TEST_6HOURS_meas = os.path.join(TEST_6HOURS, "measurements", "measurements.csv")
TEST_6HOURS_moonEph = os.path.join(TEST_6HOURS, "ephemeris", "moon_eph.csv")
TEST_6HOURS_sunEph = os.path.join(TEST_6HOURS, "ephemeris", "sun_eph.csv")
TEST_6HOURS_traj = os.path.join(TEST_6HOURS, "trajectory", "trajectory.csv")

TEST_CISLUNAR = os.path.join(TEST_DATA_DIR, "CislunarFullTraj_60secs")
TEST_CISLUNAR_meas = os.path.join(TEST_CISLUNAR, "measurements", "meas.csv")
TEST_CISLUNAR_moonEph = os.path.join(
    TEST_CISLUNAR, "ephemeris", "1min_stk_active_sampled_moon_eph.csv"
)
TEST_CISLUNAR_sunEph = os.path.join(
    TEST_CISLUNAR, "ephemeris", "1min_stk_active_sampled_sun_eph.csv"
)
TEST_CISLUNAR_traj = os.path.join(
    TEST_CISLUNAR, "trajectory", "1min_stk_active_sampled_traj.csv"
)

POS_ERROR = 1000  # position estimate can differ by 1000 km
VEL_ERROR = 1  # velocity estimate can differ by 1 km/s

POS_ERROR_6HOURS = 2500
VEL_ERROR_6HOURS = 5

EARTH_CENTER_ERROR = 20
EARTH_RADIUS_ERROR = (
    20  # Earth should be relatively easy to detect due to its blue color
)
SUN_CENTER_ERROR = 5
SUN_RADIUS_ERROR = 5  # Sun doesn't change much, which should mean better detections
MOON_CENTER_ERROR = 20
MOON_RADIUS_ERROR = 30  # Out of all bodies, Moon changes in appearance the most

# Noise in state vector provided by NASA at the start of the mission
ZERO_STARTING_NOISE = [0, 0, 0, 0, 0, 0]
SMALL_STARTING_NOISE = [1, 1, 1, 1, 1, 1]
LARGE_STARTING_NOISE = [10, 10, 10, 5, 5, 5]

# Attitude
# ATTITUDE
SPACECRAFT_MASS = 10  # spacecraft mass, kg
SPACECRAFT_HEIGHT = 0.3  # spacecraft height, meters
SPACECRAFT_WIDTH = 0.3  # spacecraft width, meters
SPACECRAFT_DEPTH = 0.1  # spacecraft depth, meters
SPACECRAFT_I_B = np.array(
    [
        [
            (1.0 / 12.0)
            * SPACECRAFT_MASS
            * (SPACECRAFT_HEIGHT ** 2.0 + SPACECRAFT_DEPTH ** 2.0),
            0.0,
            0.0,
        ],
        [
            0.0,
            (1.0 / 12.0)
            * SPACECRAFT_MASS
            * (SPACECRAFT_WIDTH ** 2.0 + SPACECRAFT_DEPTH ** 2.0),
            0.0,
        ],
        [
            0.0,
            0.0,
            (1.0 / 12.0)
            * SPACECRAFT_MASS
            * (SPACECRAFT_WIDTH ** 2.0 + SPACECRAFT_HEIGHT ** 2.0),
        ],
    ]
)  # spacecraft inertia tensor
# R * F where R = distance of cold-gas nozzle from center of mass of spacecraft
# and F = force exerted by the thruster.
TORQUE_THRUSTER = np.array([[0, 0.1, 0]]).T  # Torque exerted on the body

DAMPER_MASS = 8  # damper mass in kg
DAMPER_RADIUS = 0.1  # damper radius in meters
DAMPER_C = 0.9  # damping coefficient
DAMPER_I_D = np.array(
    [
        [(2.0 / 5.0) * DAMPER_MASS * DAMPER_RADIUS ** 2.0, 0.0, 0.0],
        [0.0, (2.0 / 5.0) * DAMPER_MASS * DAMPER_RADIUS ** 2.0, 0.0],
        [0.0, 0.0, (2.0 / 5.0) * DAMPER_MASS * DAMPER_RADIUS ** 2.0],
    ]
)

# _q0 = [0., 0., 0., 1.]                   # initial quaternion
# omega_init = [0., 0.001, 2., 0., 0., 0.]# initial angular velocity
INTEGRATION_TIMESTEP = 0.001  # integration timestep
