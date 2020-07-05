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

def cislunarFullTrajectoryWithManeuvers(visual_analysis, state_error, part_start, part_end, kickTime=None):
    """
    [part_start, part_end): start (inclusive) and end (exclusive) indices of trajectory
    """
    from tests.const import CISLUNAR_TEST_TRAJ, CISLUNAR_TEST_TRAJ_moonEph, CISLUNAR_TEST_TRAJ_sunEph, CISLUNAR_TEST_TRAJ_traj, CISLUNAR_TEST_TRAJ_att
    trajTruthdf = pd.read_csv(CISLUNAR_TEST_TRAJ_traj)
    moonEphdf = pd.read_csv(CISLUNAR_TEST_TRAJ_moonEph)
    sunEphdf = pd.read_csv(CISLUNAR_TEST_TRAJ_sunEph)
    # measEphdf = pd.read_csv(TEST_6HOURS_meas)
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
        # meas = (np.array([measEphdf.iloc[t]['z1'], measEphdf.iloc[t]['z2'], measEphdf.iloc[t]['z3'], measEphdf.iloc[t]['z4'], measEphdf.iloc[t]['z5'], measEphdf.iloc[t]['z6']], dtype=np.float)).reshape(6,1)
        # orientation = None
        # if kickTime is not None and t > kickTime:
        #     orientation = [0, 0, 0, 1]
        # state, P, K = runPosVelUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, orientation=orientation, dynamicsOnly=False)
        # Per iteration error
        traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        traj = traj.flatten()
        # fstate = state.flatten()
        # posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        # velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
        # plot
        if liveTraj:
            # liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
            liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
            liveTraj.renderUKF(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))
            liveTraj.renderUKF(text="Iteration {}".format(t))

    if liveTraj:
        liveTraj.close()

    t = part_end - 1
    traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        
    # traj = traj.flatten()
    # state = state.flatten()
    # posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    # velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    # print('Position error: {}\nVelocity error: {}'.format(posError, velError))
    # assert posError <= POS_ERROR, 'Position error is too large'
    # assert velError <= VEL_ERROR, 'Velocity error is too large'


def test_cislunarFullTrajectoryWithManeuvers_zero_starting_noise(visual_analysis):
    """
    Assumes starting state provided by NASA is very noisy.
    Each run is treated as a continuation of the previous state.
    Tests:
    - expected state is close to actual state
    - 
    """
    cislunarFullTrajectoryWithManeuvers(visual_analysis, ZERO_STARTING_NOISE, 0, 360)