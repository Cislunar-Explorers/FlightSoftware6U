import os

class MatlabTestCameraParameters:
    # Camera constants
    # Horizontal/Vertical Field of View (Degrees), Pixel Dimensions
    hFov = 62.2
    vFov = 48.8
    hPix = 3280
    vPix = 813
    #Angular Separation Between Cameras (degrees)
    dcam12 = 60
    dcam13 = -60
    dcam23 = -120

TEST_DATA_DIR = os.path.join('D:', 'OpNav')

TEST_ECLIPSEANDCRESCENTIMAGES = os.path.join(TEST_DATA_DIR, 'data','EclipseAndCrescentImages')
TEST_FIND_DATASET_IMAGE_DIR = 'images'
TEST_FIND_DATASET_CIRCLES_DIR = os.path.join('circles','circles.csv')

TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec = os.path.join(TEST_DATA_DIR, 'data','EM1_3DOF_Trajectory_June_27_2020_3600sec')
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations = 'iterations'
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory = os.path.join('trajectory','trajectory.csv')
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph = os.path.join('ephemeris','moon_eph.csv')
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph = os.path.join('ephemeris','sun_eph.csv')

TEST_C1_DISCRETIZED = os.path.join(TEST_DATA_DIR, 'data', 'c1_discretized')
TEST_C1_DISCRETIZED_meas = os.path.join(TEST_C1_DISCRETIZED, 'measurements', 'measurements.csv')
TEST_C1_DISCRETIZED_moonEph = os.path.join(TEST_C1_DISCRETIZED, 'ephemeris', 'moon_eph.csv')
TEST_C1_DISCRETIZED_sunEph = os.path.join(TEST_C1_DISCRETIZED, 'ephemeris', 'sun_eph.csv')
TEST_C1_DISCRETIZED_traj = os.path.join(TEST_C1_DISCRETIZED, 'trajectory', 'trajectory.csv')
TEST_C1_DISCRETIZED_matlab = os.path.join(TEST_C1_DISCRETIZED, 'c1_discretized_ukf_k0.csv')

TEST_6HOURS = os.path.join(TEST_DATA_DIR, 'data', '6Hours')
TEST_6HOURS_meas = os.path.join(TEST_6HOURS, 'measurements', 'measurements.csv')
TEST_6HOURS_moonEph = os.path.join(TEST_6HOURS, 'ephemeris', 'moon_eph.csv')
TEST_6HOURS_sunEph = os.path.join(TEST_6HOURS, 'ephemeris', 'sun_eph.csv')
TEST_6HOURS_traj = os.path.join(TEST_6HOURS, 'trajectory', 'trajectory.csv')

POS_ERROR = 1000 # can be off by 1000 km
VEL_ERROR = 1 # can be off by 1 km/s

EARTH_CENTER_ERROR = 20
EARTH_RADIUS_ERROR = 20 # Earth should be relatively easy to detect due to its blue color
SUN_CENTER_ERROR = 5
SUN_RADIUS_ERROR = 5 # Sun doesn't change much, which should mean better detections
MOON_CENTER_ERROR = 20
MOON_RADIUS_ERROR = 30 # Out of all bodies, Moon changes in appearance the most

# Noise in state vector provided by NASA at the start of the mission
ZERO_STARTING_NOISE = [0,0,0,0,0,0] 
SMALL_STARTING_NOISE = [1,1,1,1,1,1]
LARGE_STARTING_NOISE = [10,10,10,5,5,5]