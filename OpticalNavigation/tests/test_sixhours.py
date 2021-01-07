import pytest
import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm

from core.ukf import runTrajUKF
from tests.const import POS_ERROR_6HOURS, VEL_ERROR_6HOURS
from tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
from tests.const import MatlabTestCameraParameters
from simulations.animations import LiveTrajectoryPlot
from tests.gen_opnav_data import get6HoursBatch

################################################################
###################6 HOURS TRAJECTORY###########################
################################################################
# Testing Traj UKF on propagation data
# NOTE: trajectory is split into 6hr intervals due to impulses
# Original dataset is in minutes. In sixhours_timestep, timestep = 1
# will yield a dataset of size (part_end - part_start)
def sixhours_timestep(visual_analysis, state_error, part_start, part_end, timestep):
    """
    [part_start, part_end): start (inclusive) and end (exclusive) indices of trajectory
    """
    from tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
    d_camMeas, d_moonEph, d_sunEph, d_traj, totalIntegrationTime = get6HoursBatch(part_start, part_end, part_start, timestep, np.array([1,1,1,1,1,1]), np.array([1,1,1]), np.array([1,1,1,1]), 1, 1, 1, att_meas=False)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    state = (np.array([d_traj['x'][0], d_traj['y'][0], d_traj['z'][0], d_traj['vx'][0], d_traj['vy'][0], d_traj['vz'][0]], dtype=np.float)).reshape(6,1)
    R = np.diag(np.array(state_error, dtype=np.float))
    error = np.random.multivariate_normal(np.zeros((6,)),R).reshape(6,1)
    state = state + error

    liveTraj = None
    if visual_analysis == "True":
        print("Visual Analysis on")
        liveTraj = LiveTrajectoryPlot()
    
    # print(int(trajTruthdf.shape[0]*1 - 1))
    for t in tqdm(range( len(d_traj['x']) ), desc ='Trajectory Completion'):
        moonEph = (np.array([d_moonEph['x'][t], d_moonEph['y'][t], d_moonEph['z'][t], d_moonEph['vx'][t], d_moonEph['vy'][t], d_moonEph['vz'][t]], dtype=np.float)).reshape(1,6)
        sunEph = (np.array([d_sunEph['x'][t], d_sunEph['y'][t], d_sunEph['z'][t], d_sunEph['vx'][t], d_sunEph['vy'][t], d_sunEph['vz'][t]], dtype=np.float)).reshape(1,6)
        meas = (np.array([d_camMeas['z1'][t], d_camMeas['z2'][t], d_camMeas['z3'][t], d_camMeas['z4'][t], d_camMeas['z5'][t], d_camMeas['z6'][t]], dtype=np.float)).reshape(6,1)
        state, P, K = runTrajUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False) # used to have this, not sure why it's gone orientation=orientation
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

    t = int((part_end - part_start)/timestep - 1)
    traj = (np.array([d_traj['x'][t],
        d_traj['y'][t],
        d_traj['z'][t],
        d_traj['vx'][t],
        d_traj['vy'][t],
        d_traj['vz'][t]], dtype=np.float)).reshape(6,1)
        
    traj = traj.flatten()
    state = state.flatten()
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print(f'Position error: {posError}\nVelocity error: {velError}')
    assert posError <= POS_ERROR_6HOURS, 'Position error is too large'
    assert velError <= VEL_ERROR_6HOURS, 'Velocity error is too large'

"""
ZERO STARTING NOISE

Assumes starting state provided by NASA is accurate.
Each run is treated as a continuation of the previous state.
Tests:
- expected state is close to actual state
- 
"""
@pytest.mark.zero_noise_test
class TestZeroStartingNoiseTimestep:
    def test_interval1(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 0, 360, 1.)

    def test_interval2(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 361, 710, 1)

    def test_interval3(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 760, 1000, 1)
        
    def test_interval4(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 1100, 1400, 1)

    def test_interval5(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 0, 360, 2)

    def test_interval6(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 361, 710, 2)

    def test_interval7(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 760, 1000, 2)
        
    def test_interval8(visual_analysis):
        sixhours_timestep(visual_analysis, ZERO_STARTING_NOISE, 1100, 1400, 2)

"""
Assumes starting state provided by NASA is a bit noisy.
Each run is treated as a continuation of the previous state.
Tests:
- expected state is close to actual state
- 
"""
@pytest.mark.small_noise_test
class TestSmallStartingNoiseTimestep:
    def test_interval1(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 0, 360, 1)
    
    def test_interval2(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 361, 710, 1)
    
    def test_interval3(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 760, 1050, 1)
    
    def test_interval4(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 1100, 1400, 1)
    
    def test_interval5(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 1500, 1800, 1)

    def test_interval6(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 0, 360, 2)
    
    def test_interval7(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 361, 710, 2)
    
    def test_interval8(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 760, 1050, 2)
    
    def test_interval9(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 1100, 1400, 2)
    
    def test_interval10(visual_analysis):
        sixhours_timestep(visual_analysis, SMALL_STARTING_NOISE, 1500, 1800, 2)

"""
Assumes starting state provided by NASA is very noisy.
Each run is treated as a continuation of the previous state.
Tests:
- expected state is close to actual state
- 
"""
@pytest.mark.large_noise_test
class TestLargeStartingNoiseTimestep:
    def test_interval1(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 0, 360, 1)
    
    def test_interval2(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 361, 710, 1)
    
    def test_interval3(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 760, 1050, 1)
    
    def test_interval4(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 1100, 1400, 1)
    
    def test_interval5(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 1500, 1800, 1) # TODO: Volatile test: depends on random starting noise

    def test_interval6(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 0, 360, 2)
    
    def test_interval7(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 361, 710, 2)
    
    def test_interval8(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 760, 1050, 2)
    
    def test_interval9(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 1100, 1400, 2)
    
    def test_interval10(visual_analysis):
        sixhours_timestep(visual_analysis, LARGE_STARTING_NOISE, 1500, 1800, 2) # TODO: Volatile test: depends on random starting noise
