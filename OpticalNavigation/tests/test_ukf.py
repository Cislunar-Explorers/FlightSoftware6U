import pytest
import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm

from core.ukf import runUKF
from tests.const import POS_ERROR, VEL_ERROR
from tests.const import MatlabTestCameraParameters
from tests.animations import LiveTrajectoryPlot

def test_dynamics_model_EM1_3DOF_Trajectory_June_27_2020_3600sec():
    """
    Assumes first state vector is the initial state provided by NASA at the start of mission.
    Each run is treated as a 're-initialization' of the UKF, meaning that the delta time of 3600 secs is considered
    'too long' for the UKF to converge properly.
    Because we don't have access to the cam_meas functions, the Kalman Gain is set to 0, which means it is testing the dynamics model only.
    Tests:
    - expected state is close to actual state
    - 
    """
    from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory
    from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph
    trajTruthdf = pd.read_csv(os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory))
    moonEphdf = pd.read_csv(os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph))
    sunEphdf = pd.read_csv(os.path.join(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph))
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
    for t in range(trajTruthdf.shape[0] - 1):
        moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        state, P, K = runUKF(moonEph, sunEph, np.zeros_like(state), state, 60, P, dynamicsOnly=True)
    t = trajTruthdf.shape[0] - 1
    traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
    print(state, traj)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print(posError, velError)
    assert posError <= POS_ERROR, 'Position error is too large'
    assert velError <= VEL_ERROR, 'Velocity error is too large'

def test_ukf_c1_discretized():
    """
    Assumes first state vector is the initial state provided by NASA at the start of mission.
    Each run is treated as a 're-initialization' of the UKF, meaning that the delta time of 3600 secs is considered
    'too long' for the UKF to converge properly.
    Because we don't have access to the cam_meas functions, the Kalman Gain is set to 0, which means it is testing the dynamics model only.
    Tests:
    - expected state is close to actual state
    - 
    """
    from tests.const import TEST_C1_DISCRETIZED_meas, TEST_C1_DISCRETIZED_moonEph, TEST_C1_DISCRETIZED_sunEph, TEST_C1_DISCRETIZED_traj, TEST_C1_DISCRETIZED_matlab
    trajTruthdf = pd.read_csv(TEST_C1_DISCRETIZED_traj)
    moonEphdf = pd.read_csv(TEST_C1_DISCRETIZED_moonEph)
    sunEphdf = pd.read_csv(TEST_C1_DISCRETIZED_sunEph)
    measEphdf = pd.read_csv(TEST_C1_DISCRETIZED_meas)
    matlabUKFdf = pd.read_csv(TEST_C1_DISCRETIZED_matlab)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
    for t in tqdm(range(trajTruthdf.shape[0] - 1), desc ='Trajectory Completion'):
        moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        meas = (np.array([measEphdf.iloc[t]['z1'], measEphdf.iloc[t]['z2'], measEphdf.iloc[t]['z3'], measEphdf.iloc[t]['z4'], measEphdf.iloc[t]['z5'], measEphdf.iloc[t]['z6']], dtype=np.float)).reshape(6,1)
        state, P, K = runUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False)
    t = trajTruthdf.shape[0] - 1
    traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print('Position error: {}\nVelocity error: {}'.format(posError, velError))
    assert posError <= POS_ERROR, 'Position error is too large'
    assert velError <= VEL_ERROR, 'Velocity error is too large'

def test_ukf_6hours(visual_analysis):
    """
    Assumes first state vector is the initial state provided by NASA at the start of mission.
    Each run is treated as a 're-initialization' of the UKF, meaning that the delta time of 3600 secs is considered
    'too long' for the UKF to converge properly.
    Because we don't have access to the cam_meas functions, the Kalman Gain is set to 0, which means it is testing the dynamics model only.
    Tests:
    - expected state is close to actual state
    - 
    """
    from tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
    trajTruthdf = pd.read_csv(TEST_6HOURS_traj)
    moonEphdf = pd.read_csv(TEST_6HOURS_moonEph)
    sunEphdf = pd.read_csv(TEST_6HOURS_sunEph)
    measEphdf = pd.read_csv(TEST_6HOURS_meas)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
    
    liveTraj = None
    print(visual_analysis)
    if visual_analysis == "True":
        liveTraj = LiveTrajectoryPlot()

    for t in tqdm(range( int(trajTruthdf.shape[0]*1 - 1) ), desc ='Trajectory Completion'):
        moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        meas = (np.array([measEphdf.iloc[t]['z1'], measEphdf.iloc[t]['z2'], measEphdf.iloc[t]['z3'], measEphdf.iloc[t]['z4'], measEphdf.iloc[t]['z5'], measEphdf.iloc[t]['z6']], dtype=np.float)).reshape(6,1)
        state, P, K = runUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False)
        # Per iteration error
        traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        traj = traj.flatten()
        fstate = state.flatten()
        posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        # plot
        if liveTraj:
            liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
            liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
            liveTraj.render()

    t = trajTruthdf.shape[0] - 1
    traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print('Position error: {}\nVelocity error: {}'.format(posError, velError))
    assert posError <= POS_ERROR, 'Position error is too large'
    assert velError <= VEL_ERROR, 'Velocity error is too large'
