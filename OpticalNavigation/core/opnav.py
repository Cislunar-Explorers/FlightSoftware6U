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
    # Step 1: Convert video clips into list of measurements
    # TODO: Finalize video format and conversion to frames
    measList = [np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])] # earth, moon, sun
    gyroList = [[0], [0], [0], [0], [0], [0]] # omegaX, omegaY, omegaZ, biasX, biasY, biasZ
