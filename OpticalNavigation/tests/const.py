import os

TEST_ECLIPSEANDCRESCENTIMAGES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data','EclipseAndCrescentImages')
TEST_FIND_DATASET_IMAGE_DIR = 'images'
TEST_FIND_DATASET_CIRCLES_DIR = os.path.join('circles','circles.csv')

TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data','EM1_3DOF_Trajectory_June_27_2020_3600sec')
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations = 'iterations'
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory = os.path.join('trajectory','trajectory.csv')
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph = os.path.join('ephemeris','moon_eph.csv')
TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph = os.path.join('ephemeris','sun_eph.csv')

POS_ERROR = 1000 # can be off by 1000 km
VEL_ERROR = 1 # can be off by 1 km/s

EARTH_CENTER_ERROR = 20
EARTH_RADIUS_ERROR = 20 # Earth should be relatively easy to detect due to its blue color
SUN_CENTER_ERROR = 5
SUN_RADIUS_ERROR = 5 # Sun doesn't change much, which should mean better detections
MOON_CENTER_ERROR = 20
MOON_RADIUS_ERROR = 30 # Out of all bodies, Moon changes in appearance the most