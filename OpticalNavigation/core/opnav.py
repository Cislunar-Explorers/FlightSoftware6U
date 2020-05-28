from core.acquisition import startAcquisition, readOmega
from core.cam_meas import cameraMeasurements
from core.ukf import runPosVelUKF
from core.attitude import runAttitudeUKFWithKick
import numpy as np
import traceback
import pandas as pd

"""
Begin OpNav acquisition and storing process. The system will record videos from
the three cameras onboard and store them on the SD card as video format. It will
record gyroscope measurements and store them in a table on the SD card.
"""
def observe():
    # obtain latest angular velocity (account for bias)
    # configure camera parameters accordingly
    # decide on video time
    # record video from camera 1
    # record video from camera 2
    # record video from camera 3
    # record angular velocity from gyro (N times)
    # Extract measurements from video frames
    # Store measurement vector and angular velocity vector in database
    # Repeat until desired number of camera measurements have been obtained
    #   Low small angular velocities and cold-gas kick, Att-UKF converges with a few hundered.
    pass

"""
Begin OpNav processing sequence for obtaining position, velocity and attitude estimates.
This function is triggered once the "observer" mode of spacecraft has finished accumulating
enough data from its cameras and gyroscope. The input will be the start and end time for
the duration of the observer mode being on, ephemiris tables, starting states, camera
measurements, gyroscope measurements, main thruster fire sequences and cold-gas thruster 
fire sequences at specified times. The output will be the current position, velocity, 
attitude and angular velocity estimates. 
"""
def process(startTime, duration, moonTable, sunTable, initPosVel, initOmega, initQuat, camDir, gyroCSV):
    """
    [startTime] start time index for ephemeris tables
    [duration] how long the observation mode lasted
    [moonTable, sunTable] full ephemeris tables
    [initPosVel] previous position and velocity estimate
    [initOmega] previous angular velocity estimate
    [initQuat] previous orientation quaternion estimate
    [camDir] directory of the camera video clips
    [gyroCSV] location of the gyro measurements csv file
    Returns:
    [newPos, newVel, newOmega, newQuat]
    """
    # Pull entries from database and re-format in desired tables
    # Pull ephemeris tables and identify start and end times
    # Pull main thrust fire times
    # Organize measurements into groups, and batch them by thrust fires.
    # Filtering Step:
    #   Feed a batch into Traj-UKF and obtain position estimates history
    #   Propagate position to each of the N gyro times using estimated velocity, for each pos-vel estimate in history
    #   Calculate Sun, Moon and Earth positions in body frame
    #   Feed new measurement vectors and gyro vectors into Att-UKF.
    #   (Next iteration of Traj-UKF will use quaternion history for thrust fire dynamics propagation)
    pass

