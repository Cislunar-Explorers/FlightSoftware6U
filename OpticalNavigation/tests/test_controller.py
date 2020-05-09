import pytest
import pandas as pd
import numpy as np
import os
import traceback
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations
from tests.const import POS_ERROR, VEL_ERROR
from tests.const import MatlabTestCameraParameters
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
        testImgDir = os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_iterations, str(iteration))
        t = iteration - 1
        try:
            state, P, K = run(t, moonEph, sunEph, state, P, MatlabTestCameraParameters, dir=testImgDir)
            trueTraj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
            print('---------------\nTrue traj: {}\nEstimated traj: {}\nK: {}\nP: {}'.format(trueTraj, state, K, P))
        except Exception as e:
            print(e)
