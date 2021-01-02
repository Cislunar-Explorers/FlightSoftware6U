from numpy.matrixlib.defmatrix import matrix
import pytest
import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm
from datetime import datetime, timedelta
from time import sleep
# from sqlalchemy.orm import session

from OpticalNavigation.core.ukf import runTrajUKF
from OpticalNavigation.tests.const import POS_ERROR_6HOURS, VEL_ERROR_6HOURS
from OpticalNavigation.tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
from OpticalNavigation.tests.const import MatlabTestCameraParameters
from OpticalNavigation.simulations.animations import LiveTrajectoryPlot
from OpticalNavigation.tests.gen_opnav_data import get6HoursBatch
from OpticalNavigation.tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
from OpticalNavigation.core.const import AttitudeEstimateOutput, AttitudeStateVector, CovarianceMatrix, OPNAV_EXIT_STATUS, QuaternionVector, TrajUKFConstants
from FlightSoftware.utils.db import OpNavEphemerisModel, OpNavCameraMeasurementModel, OpNavPropulsionModel, OpNavGyroMeasurementModel
from FlightSoftware.utils.db import create_sensor_tables_from_path, OpNavTrajectoryStateModel, OpNavAttitudeStateModel
import OpticalNavigation.core.opnav as opnav

SQL_PREFIX = "sqlite:///"
sql_path = SQL_PREFIX + os.path.join("D:", "OpNav", "db", "satellite-db.sqlite")

def setup_function(function):
    print("setup...")
    # Reset databases
    create_session = create_sensor_tables_from_path(sql_path)
    session = create_session()
    session.query(OpNavTrajectoryStateModel).delete()
    assert len(session.query(OpNavTrajectoryStateModel).all()) == 0
    session.query(OpNavAttitudeStateModel).delete()
    assert len(session.query(OpNavAttitudeStateModel).all()) == 0
    session.query(OpNavEphemerisModel).delete()
    assert len(session.query(OpNavEphemerisModel).all()) == 0
    session.query(OpNavCameraMeasurementModel).delete()
    assert len(session.query(OpNavCameraMeasurementModel).all()) == 0
    session.query(OpNavPropulsionModel).delete()
    assert len(session.query(OpNavPropulsionModel).all()) == 0
    session.query(OpNavGyroMeasurementModel).delete()
    assert len(session.query(OpNavGyroMeasurementModel).all()) == 0
    session.commit()

def teardown_function(function):
    print("teardown...")

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

def sixhours_db(mocker, state_error, part_start, part_end, timestep):
    """
    Template function for the Six Hours Trajectory UKF unit tests.
    Data:
        - noise of magnitude [state_error] in starting estimate
        - no noise in measurements
        - no added noise in proceeding state estimates
        - init state was taken seconds prior to current measurements
        - ephemerides and measurements taken at same timesteps
    """
    create_session = create_sensor_tables_from_path(sql_path)
    session = create_session()
    time = datetime.now()
    ## PREPARE DATABASES ##
    d_camMeas, d_moonEph, d_sunEph, d_traj, totalIntegrationTime = get6HoursBatch(part_start, part_end, part_start, timestep, np.array([1,1,1,1,1,1]), np.array([1,1,1]), np.array([1,1,1,1]), 1, 1, 1, att_meas=False)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    R = np.diag(np.array(state_error, dtype=np.float))
    # init state
    state = (np.array([d_traj['x'][0], d_traj['y'][0], d_traj['z'][0], d_traj['vx'][0], d_traj['vy'][0], d_traj['vz'][0]], dtype=np.float)).reshape(6,1)
    print(state)
    state[0:3] = state[0:3] - 200 * state[3:6]
    print(state)
    error = np.random.multivariate_normal(np.zeros((6,)),R).reshape(6,1)
    state = state + error
    state = state.flatten()
    entries = [
        OpNavTrajectoryStateModel.from_tuples(
            position=(state[0], state[1], state[2]), 
            velocity=(state[3], state[4], state[5]), 
            P=TrajUKFConstants.P0.reshape(6,6), 
            time=time+timedelta(0,-200)), # initial state used by propulsion step, output used by propagation step
        OpNavAttitudeStateModel.from_tuples(
            quat=(0., 0., 0., 0.), 
            rod_params=(0., 0., 0.), 
            biases=(0.,0.,0.,), 
            P=np.zeros((6,6)), 
            time=time), # dummy attitude states
    ]
    measurements = []
    # save epehemerides to db and prepare measurements
    # liveTraj = None
    # if visual_analysis == "True":
    #     print("Visual Analysis on")
    #     liveTraj = LiveTrajectoryPlot()

    for t in tqdm(range( len(d_traj['x']) ), desc ='Trajectory Completion'):
        new_time = time+timedelta(0,timestep*t)
        measurements.append(
            OpNavCameraMeasurementModel.from_tuples(
                measurement=(d_camMeas['z1'][t], d_camMeas['z2'][t], d_camMeas['z3'][t], d_camMeas['z4'][t], d_camMeas['z5'][t], d_camMeas['z6'][t]), 
                time=new_time), # camera measurement taken now, uses state 
        )
        entries.extend([
            OpNavEphemerisModel.from_tuples(
                sun_eph=(d_sunEph['x'][t], d_sunEph['y'][t], d_sunEph['z'][t], d_sunEph['vx'][t], d_sunEph['vy'][t], d_sunEph['vz'][t]), 
                moon_eph=(d_moonEph['x'][t], d_moonEph['y'][t], d_moonEph['z'][t], d_moonEph['vx'][t], d_moonEph['vy'][t], d_moonEph['vz'][t]), 
                time=new_time),
        ])
    
    # save ephemerides
    session.bulk_save_objects(entries)
    session.commit()
        # state, P, K = runTrajUKF(moonEph, sunEph, meas, state, 60, P, MatlabTestCameraParameters, dynamicsOnly=False) # used to have this, not sure why it's gone orientation=orientation
        # # Per iteration error
        # traj = (np.array([d_traj['x'][t], d_traj['y'][t], d_traj['z'][t], d_traj['vx'][t], d_traj['vy'][t], d_traj['vz'][t]], dtype=np.float)).reshape(6,1)
        # traj = traj.flatten()
        # fstate = state.flatten()
        # posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        # velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
        # # plot
        # if liveTraj:
        #     liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
        #     liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
        #     liveTraj.renderUKF(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))
    
    # setup mocks
    def observe_mock(session, gyro_count):
        curr_meas = measurements[0]
        curr_time = curr_meas.time_retrieved
        session.bulk_save_objects([
            curr_meas,
            OpNavGyroMeasurementModel.from_tuples(
                measurement=(0., 0., 0.), 
                time=curr_time), # dummy gyro measurement 1
            OpNavGyroMeasurementModel.from_tuples(
                measurement=(0., 0., 0.), 
                time=curr_time+timedelta(0, 1)), # dummy gyro measurement 2
        ])
        session.commit()
        measurements.pop(0)
        print(f'Measurements remaining----------{len(measurements)}')
        return OPNAV_EXIT_STATUS.SUCCESS
    mocker.patch('OpticalNavigation.core.opnav.__observe', side_effect=observe_mock)

    def process_propulsion_events_mock(*args):
        return OPNAV_EXIT_STATUS.SUCCESS
    mocker.patch('OpticalNavigation.core.opnav.__process_propulsion_events', side_effect=process_propulsion_events_mock)

    def run_attitude_ukf_mock(*args):
        print(f'-----------MOCK-----------')
        return AttitudeEstimateOutput(new_state=AttitudeStateVector(0,0,0,0,0,0),
                                                    new_P=CovarianceMatrix(matrix=np.zeros((6,6))), 
                                                    new_quat=QuaternionVector(0, 0, 0, 0))
    mocker.patch('OpticalNavigation.core.attitude.runAttitudeUKF', side_effect=run_attitude_ukf_mock)
    
    # start opnav system
    status = opnav.start(sql_path=sql_path, num_runs=len(d_traj['x']), gyro_count=2)

    # verification
    # len(d_traj['x']) new entries in the trajectory state table
    # start() should succeed
    # position and velocity errors should be within bounds

def test_6hours_db(mocker):
    sixhours_db(mocker, ZERO_STARTING_NOISE, 0, 360, 1.)

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
