import pytest
import pandas as pd
import numpy as np
import os
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations
from tests.const import POS_ERROR, VEL_ERROR
from core.const import ACQUIRED_IMGS_DIR
from core.controller import run

def test_controller_EM1_3DOF_Trajectory_June_27_2020_3600sec():
    """
    Assumes first state vector is the initial state provided by NASA at the start of mission.
    Each run is treated as a 're-initialization' of the UKF, meaning that the delta time of 3600 secs is considered
    'too long' for the UKF to converge properly.
    Because we don't have access to the cam_meas functions, the Kalman Gain is set to 0, which means it is testing the dynamics model only.
    Tests:
    - expected state is close to actual state
    - 
    """
    trajTruthdf = pd.read_csv(os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory))
    moonEph = pd.read_csv(os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph)).to_numpy()
    sunEph = pd.read_csv(os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph)).to_numpy()
    state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State

    for iteration in range(1, 142):
        ACQUIRED_IMGS_DIR = '{}{}\\{}'.format(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations, iteration)
        t = iteration - 1
        state, P, K = run(t, moonEph, sunEph, state, P)
        
        trueTraj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        print('---------------\nTrue traj: {}\nEstimated traj: {}\nK: {}\nP: {}'.format(trueTraj, state, K, P))
    # for t in range(trajTruthdf.shape[0] - 1):
    #     moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
    #     sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
    #     P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    #     state, pNew, K = runUKF(moonEph, sunEph, np.zeros_like(state), state, 60, P, dynamicsOnly=True)

    # t = trajTruthdf.shape[0] - 1
    # traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
    # print(state, traj)
        
    # traj = traj.flatten()
    # state = state.flatten()
    # posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    # velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    # print(posError, velError)
    # self.assertLessEqual(posError, self.POS_ERROR)
    # self.assertLessEqual(velError, self.VEL_ERROR)
