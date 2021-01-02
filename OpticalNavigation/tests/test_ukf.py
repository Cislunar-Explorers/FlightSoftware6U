from OpticalNavigation.tests.const import CesiumTestCameraParameters
from OpticalNavigation.core.const import CameraMeasurementVector, CovarianceMatrix, EphemerisVector, TrajectoryEstimateOutput, TrajectoryStateVector
import pytest
import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm

from core.ukf import runTrajUKF
from tests.const import POS_ERROR, VEL_ERROR
from tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
from tests.const import MatlabTestCameraParameters
from simulations.animations import LiveTrajectoryPlot
from tests.gen_opnav_data import get6HoursBatch

# def test_ukf_c1_discretized_zero_starting_noise(visual_analysis):
#     """
#     Assumes starting state provided by NASA is accurate.
#     Each run is treated as a continuation of the previous state.
#     Tests:
#     - expected state is close to actual state
#     """
#     c1_discretized(visual_analysis=visual_analysis, state_error=ZERO_STARTING_NOISE)

# # DOES NOT CONVERGE
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

# def c1_discretized(visual_analysis, state_error):
#     from tests.const import TEST_C1_DISCRETIZED_meas, TEST_C1_DISCRETIZED_moonEph, TEST_C1_DISCRETIZED_sunEph, TEST_C1_DISCRETIZED_traj, TEST_C1_DISCRETIZED_matlab
#     trajTruthdf = pd.read_csv(TEST_C1_DISCRETIZED_traj)
#     moonEphdf = pd.read_csv(TEST_C1_DISCRETIZED_moonEph)
#     sunEphdf = pd.read_csv(TEST_C1_DISCRETIZED_sunEph)
#     measEphdf = pd.read_csv(TEST_C1_DISCRETIZED_meas)
#     matlabUKFdf = pd.read_csv(TEST_C1_DISCRETIZED_matlab)
#     P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
#     state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
#     R = np.diag(np.array(state_error, dtype=np.float))
#     error = np.random.multivariate_normal(np.zeros((6,)),R).reshape(6,1)
#     state = state + error

#     liveTraj = None
#     if visual_analysis == "True":
#         liveTraj = LiveTrajectoryPlot()

#     for t in tqdm(range(trajTruthdf.shape[0] - 1), desc ='Trajectory Completion'):
#         moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
#         sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
#         meas = (np.array([measEphdf.iloc[t]['z1'], measEphdf.iloc[t]['z2'], measEphdf.iloc[t]['z3'], measEphdf.iloc[t]['z4'], measEphdf.iloc[t]['z5'], measEphdf.iloc[t]['z6']], dtype=np.float)).reshape(6,1)
#         state, P, K = runTrajUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False)
#         # Per iteration error
#         traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
#         traj = traj.flatten()
#         fstate = state.flatten()
#         posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
#         velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
#         # plot
#         if liveTraj:
#             liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
#             liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
#             liveTraj.render(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))
    
#     if liveTraj:
#         liveTraj.close()
    
#     t = trajTruthdf.shape[0] - 1
#     traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        
#     traj = traj.flatten()
#     state = state.flatten()
#     posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
#     velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
#     print(f'Position error: {posError}\nVelocity error: {velError}')
#     assert posError <= POS_ERROR, 'Position error is too large'
#     assert velError <= VEL_ERROR, 'Velocity error is too large'

def test_cislunar_ukf(mocker):
    cislunar1_timestep(False, state_error=LARGE_STARTING_NOISE)

def cislunar1_timestep(visual_analysis, state_error, kickTime=None):
    """
    [part_start, part_end): start (inclusive) and end (exclusive) indices of trajectory
    """
    from OpticalNavigation.tests.const import TEST_CISLUNAR_meas, TEST_CISLUNAR_moonEph, TEST_CISLUNAR_sunEph, TEST_CISLUNAR_traj
    # d_camMeas, d_moonEph, d_sunEph, d_traj, totalIntegrationTime = get6HoursBatch(part_start, part_end, part_start, timestep, np.array([1,1,1,1,1,1]), np.array([1,1,1]), np.array([1,1,1,1]), 1, 1, 1, att_meas=False)
    d_traj = pd.read_csv(TEST_CISLUNAR_traj).to_dict()
    d_moonEph = pd.read_csv(TEST_CISLUNAR_moonEph).to_dict()
    d_sunEph = pd.read_csv(TEST_CISLUNAR_sunEph).to_dict()
    d_camMeas = pd.read_csv(TEST_CISLUNAR_meas).to_dict()
    iterations = min(len(d_traj['x']), len(d_moonEph['x']), len(d_sunEph['x']), len(d_camMeas['Z1']))
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([d_traj['x'][0], d_traj['y'][0], d_traj['z'][0], d_traj['vx'][0], d_traj['vy'][0], d_traj['vz'][0]], dtype=np.float)).reshape(6,1)
    R = np.diag(np.array(state_error, dtype=np.float))
    error = np.random.multivariate_normal(np.zeros((6,)),R).reshape(6,1)
    state = state + error

    liveTraj = None
    if visual_analysis == "True":
        liveTraj = LiveTrajectoryPlot()

    # print(int(trajTruthdf.shape[0]*1 - 1))
    for t in tqdm(range(iterations), desc ='Trajectory Completion'):
        moonEph = EphemerisVector(x_pos=d_moonEph['x'][t], y_pos=d_moonEph['y'][t], z_pos=d_moonEph['z'][t], x_vel=d_moonEph['vx'][t], y_vel=d_moonEph['vy'][t], z_vel=d_moonEph['vz'][t])
        sunEph = EphemerisVector(x_pos=d_sunEph['x'][t], y_pos=d_sunEph['y'][t], z_pos=d_sunEph['z'][t], x_vel=d_sunEph['vx'][t], y_vel=d_sunEph['vy'][t], z_vel=d_sunEph['vz'][t])
        meas = CameraMeasurementVector(ang_em=d_camMeas['Z1'][t], ang_es=d_camMeas['Z2'][t], ang_ms=d_camMeas['Z3'][t], e_dia=d_camMeas['Z4'][t], m_dia=d_camMeas['Z5'][t], s_dia=d_camMeas['Z6'][t])
        orientation = None
        if kickTime is not None and t > kickTime:
            orientation = [0, 0, 0, 1]
        stateEstimate:TrajectoryEstimateOutput = runTrajUKF(moonEph, sunEph, meas, TrajectoryStateVector.from_numpy_array(state=state), 60, CovarianceMatrix(matrix=P), CesiumTestCameraParameters, dynamicsOnly=False) # used to have this, not sure why it's gone orientation=orientation
        state = stateEstimate.new_state.data
        P = stateEstimate.new_P.data
        K = stateEstimate.K.data
        # Per iteration error
        traj = (np.array([d_traj['x'][t], d_traj['y'][t], d_traj['z'][t], d_traj['vx'][t], d_traj['vy'][t], d_traj['vz'][t]], dtype=np.float)).reshape(6,1)
        traj = traj.flatten()
        fstate = state.flatten()
        posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
        # plot
        if liveTraj:
            liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
            liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
            liveTraj.renderUKF(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))

    if liveTraj:
        liveTraj.close()

    t = iterations - 1
    traj = (np.array([d_traj['x'][t], d_traj['y'][t], d_traj['z'][t], d_traj['vx'][t], d_traj['vy'][t], d_traj['vz'][t]], dtype=np.float)).reshape(6,1)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print(f'Position error: {posError}\nVelocity error: {velError}')
    assert posError <= POS_ERROR, 'Position error is too large'
    assert velError <= VEL_ERROR, 'Velocity error is too large'