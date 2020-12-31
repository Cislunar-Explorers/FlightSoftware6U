from core.acquisition import startAcquisition, readOmega
from core.cam_meas import cameraMeasurements
from core.ukf import runTrajUKF
from core.attitude import runAttitudeUKF
from core.sense import select_camera, record_video, record_gyro
from core.preprocess import rolling_shutter, rect_to_stereo_proj
from core.find import find
import numpy as np
import traceback
import pandas as pd
from utils.db import create_sensor_tables_from_path, OpNavCoordinatesModel
from utils.constants import DB_FILE
from datetime import datetime

"""
Entry point into OpNav. This method calls observe() and process().
"""


def start():
    """
    Confirms if estimates were computed successfully and if 
    they were added tp the appropriate databases.
    Return:
    opnav_exit_status: 0 (success), ...
    """
    create_session = create_sensor_tables_from_path(DB_FILE)
    session = create_session()
    new_entry = OpNavCoordinatesModel(
        time_retrieved=datetime.now(),
        velocity_x=0.,
        velocity_y=0.,
        velocity_z=0.,
        position_x=0.,
        position_y=0.,
        position_z=0.,
        attitude_q1=0.,
        attitude_q2=0.,
        attitude_q3=0.,
        attitude_q4=0.,
        attitude_rod1=0.,
        attitude_rod2=0.,
        attitude_rod3=0.,
        attitude_b1=0.,
        attitude_b2=0.,
        attitude_b3=0.
    )
    session.add(new_entry)
    session.commit()
    # Check db entries
    entries = session.query(OpNavCoordinatesModel).all()
    for entry in entries:
        print(entry)
    return 0

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
def process(batch, initTrajState, traj_P, cameraParams, att_P, initAttitudeState, initQuaternionState, gyroVars):
    """
    For an example of input format, see tests.test_pipeline.testRandomData()
    [batch]: dictionary with camera measurement, gyro measurement, and thrust acceleration measurement data
    [initTrajState]: (6,1) np array pos,vel from last run of traj UKF
    [traj_P]: (6,6) np matrix representing traj UKF covariance matrix
    [cameraParams]: data camera parameters
    [att_P]: (6,6) np matrix representing att UKF covariance matrix 
    [initAttitudeState]: (6,1) np array initial attitude State
    [initQuaterionState]: (4,1) np array initial quaternion state (could be a guess)
    [gyroVars]: (gyro_sigma, gyro_sample_rate, Q, R) where
                [gyro_sigma]: float
                [gyro_sample_rate]: average sample time elapsed between each gyro measurement (seconds)
                [Q]: (6,6) np matrix
                [R]: (6,6) np matrix
    Returns:
    [newPos, newVel, newOmega, newQuat, trajP, trajK]
    """
    trajState = initTrajState
    traj_K = None
    attState = initAttitudeState
    quat = initQuaternionState
    
    for ib, b in enumerate(batch):
        print(ib)
        # Run Trajectory UKF
        if b['main_thrust']['fire'] is False:
            main_thrust_info = None
        else:
            main_thrust_info = {
                'kick_orientation': quat/np.linalg.norm(quat),
                'acceleration_magnitude': b['main_thrust']['thrust_mag']
            }
            # Aq = attitudeMatrix(q)
            # # X axis of spacecraft
            # local_vector = np.array([1, 0, 0, 0]) # last 0 is padding
            # X_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
        trajState, traj_P, traj_K = runTrajUKF(b['ephemeris']['moon'], 
                                       b['ephemeris']['sun'], 
                                       b['cam_meas'], 
                                       trajState,
                                       b['cam_dt'], 
                                       traj_P, 
                                       cameraParams,
                                       main_thrust_info=main_thrust_info,
                                       dynamicsOnly=False)
        # Prepare Gyro data
        totalIntegrationTime = 0
        omegas = np.zeros((len(b['gyro_meas']),3))
        biases = np.zeros((len(b['gyro_meas']),3))
        estimatedSatState = np.zeros((len(b['gyro_meas']),3))
        moonPos = np.zeros((len(b['gyro_meas']),3))
        sunPos = np.zeros((len(b['gyro_meas']),3))

        tempSatState = trajState.flatten()
        tempMoonState = b['ephemeris']['moon'].flatten()
        tempSunState = b['ephemeris']['sun'].flatten()
        timeline = [0]*(len(b['gyro_meas']))
        prevTime = 0
        for i in range(omegas.shape[0]):
            om = b['gyro_meas'][i]['omega'].flatten()
            bi = b['gyro_meas'][i]['bias'].flatten()
            omegas[i][0] = om[0]
            omegas[i][1] = om[1]
            omegas[i][2] = om[2]
            biases[i][0] = bi[0]
            biases[i][1] = bi[1]
            biases[i][2] = bi[2]
            estimatedSatState[i][0] = tempSatState[0]
            estimatedSatState[i][1] = tempSatState[1]
            estimatedSatState[i][2] = tempSatState[2]
            moonPos[i][0] = tempMoonState[0]
            moonPos[i][1] = tempMoonState[1]
            moonPos[i][2] = tempMoonState[2]
            sunPos[i][0] = tempSunState[0]
            sunPos[i][1] = tempSunState[1]
            sunPos[i][2] = tempSunState[2]
            prevTime += b['gyro_meas'][i]['dt']
            timeline[i] = prevTime


        att_P, quat, attState = runAttitudeUKF(b['cam_dt'], 
                                                       gyroVars, 
                                                       att_P, 
                                                       attState, 
                                                       quat, 
                                                       omegas, 
                                                       biases, 
                                                       estimatedSatState, 
                                                       moonPos, 
                                                       sunPos, 
                                                       timeline,
                                                       singleIteration=True)

    
    # END

