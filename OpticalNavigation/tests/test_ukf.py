import pytest
import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm

from core.ukf import runPosVelUKF
from tests.const import POS_ERROR, VEL_ERROR
from tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
from tests.const import MatlabTestCameraParameters
from tests.animations import LiveTrajectoryPlot

def test_ukf_c1_discretized_zero_starting_noise(visual_analysis):
    """
    Assumes starting state provided by NASA is accurate.
    Each run is treated as a continuation of the previous state.
    Tests:
    - expected state is close to actual state
    """
    c1_discretized(visual_analysis=visual_analysis, state_error=ZERO_STARTING_NOISE)

# DOES NOT CONVERGE
# def test_ukf_c1_discretized_small_starting_noise(visual_analysis):
#     """
#     Assumes starting state provided by NASA is a bit noisy.
#     Each run is treated as a continuation of the previous state.
#     Tests:
#     - should diverge due to small size of trajectory
#     """
#     c1_discretized(visual_analysis=visual_analysis, state_error=SMALL_STARTING_NOISE)

# def test_ukf_c1_discretized_large_starting_noise(visual_analysis):
#     """
#     Assumes starting state provided by NASA is very noisy.
#     Each run is treated as a continuation of the previous state.
#     Tests:
#     - should diverge due to small size of trajectory
#     """
#     c1_discretized(visual_analysis=visual_analysis, state_error=LARGE_STARTING_NOISE)

def c1_discretized(visual_analysis, state_error):
    from tests.const import TEST_C1_DISCRETIZED_meas, TEST_C1_DISCRETIZED_moonEph, TEST_C1_DISCRETIZED_sunEph, TEST_C1_DISCRETIZED_traj, TEST_C1_DISCRETIZED_matlab
    trajTruthdf = pd.read_csv(TEST_C1_DISCRETIZED_traj)
    moonEphdf = pd.read_csv(TEST_C1_DISCRETIZED_moonEph)
    sunEphdf = pd.read_csv(TEST_C1_DISCRETIZED_sunEph)
    measEphdf = pd.read_csv(TEST_C1_DISCRETIZED_meas)
    matlabUKFdf = pd.read_csv(TEST_C1_DISCRETIZED_matlab)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
    R = np.diag(np.array(state_error, dtype=np.float))
    error = np.random.multivariate_normal(np.zeros((6,)),R).reshape(6,1)
    state = state + error

    liveTraj = None
    if visual_analysis == "True":
        liveTraj = LiveTrajectoryPlot()

    for t in tqdm(range(trajTruthdf.shape[0] - 1), desc ='Trajectory Completion'):
        moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        meas = (np.array([measEphdf.iloc[t]['z1'], measEphdf.iloc[t]['z2'], measEphdf.iloc[t]['z3'], measEphdf.iloc[t]['z4'], measEphdf.iloc[t]['z5'], measEphdf.iloc[t]['z6']], dtype=np.float)).reshape(6,1)
        state, P, K = runPosVelUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False)
        # Per iteration error
        traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        traj = traj.flatten()
        fstate = state.flatten()
        posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
        # plot
        if liveTraj:
            liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
            liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
            liveTraj.render(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))
    
    if liveTraj:
        liveTraj.close()
    
    t = trajTruthdf.shape[0] - 1
    traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print('Position error: {}\nVelocity error: {}'.format(posError, velError))
    assert posError <= POS_ERROR, 'Position error is too large'
    assert velError <= VEL_ERROR, 'Velocity error is too large'

################################################################
###################6 HOURS TRAJECTORY###########################
################################################################
# NOTE: trajectory is split into parts because it contains impulses

def test_ukf_6hours_zero_starting_noise(visual_analysis):
    """
    Assumes starting state provided by NASA is accurate.
    Each run is treated as a continuation of the previous state.
    Tests:
    - expected state is close to actual state
    - 
    """
    sixhours(visual_analysis, ZERO_STARTING_NOISE, 0, 360)
    sixhours(visual_analysis, ZERO_STARTING_NOISE, 361, 710)
    sixhours(visual_analysis, ZERO_STARTING_NOISE, 760, 1000)
    sixhours(visual_analysis, ZERO_STARTING_NOISE, 1100, 1400)
    sixhours(visual_analysis, ZERO_STARTING_NOISE, 1500, 1700)

def test_ukf_6hours_small_starting_noise(visual_analysis):
    """
    Assumes starting state provided by NASA is a bit noisy.
    Each run is treated as a continuation of the previous state.
    Tests:
    - expected state is close to actual state
    - 
    """
    sixhours(visual_analysis, SMALL_STARTING_NOISE, 0, 360)
    sixhours(visual_analysis, SMALL_STARTING_NOISE, 361, 710)
    sixhours(visual_analysis, SMALL_STARTING_NOISE, 760, 1050)
    sixhours(visual_analysis, SMALL_STARTING_NOISE, 1100, 1400)
    sixhours(visual_analysis, SMALL_STARTING_NOISE, 1500, 1800)

def test_ukf_6hours_large_starting_noise(visual_analysis):
    """
    Assumes starting state provided by NASA is very noisy.
    Each run is treated as a continuation of the previous state.
    Tests:
    - expected state is close to actual state
    - 
    """
    sixhours(visual_analysis, LARGE_STARTING_NOISE, 0, 360)
    sixhours(visual_analysis, LARGE_STARTING_NOISE, 361, 710)
    sixhours(visual_analysis, LARGE_STARTING_NOISE, 760, 1050)
    sixhours(visual_analysis, LARGE_STARTING_NOISE, 1100, 1400)
    sixhours(visual_analysis, LARGE_STARTING_NOISE, 1500, 1800) # TODO: Volatile test: depends on random starting noise

def sixhours(visual_analysis, state_error, part_start, part_end):
    """
    [part_start, part_end): start (inclusive) and end (exclusive) indices of trajectory
    """
    from tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
    trajTruthdf = pd.read_csv(TEST_6HOURS_traj)
    moonEphdf = pd.read_csv(TEST_6HOURS_moonEph)
    sunEphdf = pd.read_csv(TEST_6HOURS_sunEph)
    measEphdf = pd.read_csv(TEST_6HOURS_meas)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([trajTruthdf.iloc[part_start]['x'], trajTruthdf.iloc[part_start]['y'], trajTruthdf.iloc[part_start]['z'], trajTruthdf.iloc[part_start]['vx'], trajTruthdf.iloc[part_start]['vy'], trajTruthdf.iloc[part_start]['vz']], dtype=np.float)).reshape(6,1)
    R = np.diag(np.array(state_error, dtype=np.float))
    error = np.random.multivariate_normal(np.zeros((6,)),R).reshape(6,1)
    state = state + error

    liveTraj = None
    if visual_analysis == "True":
        liveTraj = LiveTrajectoryPlot()

    # print(int(trajTruthdf.shape[0]*1 - 1))
    for t in tqdm(range( part_start, part_end ), desc ='Trajectory Completion'):
        moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
        meas = (np.array([measEphdf.iloc[t]['z1'], measEphdf.iloc[t]['z2'], measEphdf.iloc[t]['z3'], measEphdf.iloc[t]['z4'], measEphdf.iloc[t]['z5'], measEphdf.iloc[t]['z6']], dtype=np.float)).reshape(6,1)
        state, P, K = runUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False)
        # Per iteration error
        traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        traj = traj.flatten()
        fstate = state.flatten()
        posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
        # plot
        if liveTraj:
            liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
            liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
            liveTraj.render(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))

    if liveTraj:
        liveTraj.close()

    t = part_end - 1
    traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print('Position error: {}\nVelocity error: {}'.format(posError, velError))
    assert posError <= POS_ERROR, 'Position error is too large'
    assert velError <= VEL_ERROR, 'Velocity error is too large'
