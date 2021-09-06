from numpy.matrixlib.defmatrix import matrix
import pytest
import pandas as pd
import numpy as np
import math
import os
from sqlalchemy.sql.expression import true
from tqdm import tqdm
from datetime import datetime, timedelta
from time import sleep
# from sqlalchemy.orm import session

from core.ukf import runTrajUKF
from tests.const import POS_ERROR_6HOURS, VEL_ERROR_6HOURS
from tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
from tests.const import MatlabTestCameraParameters
from simulations.animations import LiveTrajectoryPlot
from tests.gen_opnav_data import get6HoursBatch
from tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
from core.const import AttitudeEstimateOutput, AttitudeStateVector, CameraMeasurementVector, CovarianceMatrix, \
    EphemerisVector, Matrix6x6, OPNAV_EXIT_STATUS, QuaternionVector, TrajUKFConstants, TrajectoryEstimateOutput, \
    TrajectoryStateVector
from FlightSoftware.utils.db import OpNavEphemerisModel, OpNavCameraMeasurementModel, OpNavPropulsionModel, \
    OpNavGyroMeasurementModel
from FlightSoftware.utils.db import create_sensor_tables_from_path, OpNavTrajectoryStateModel, OpNavAttitudeStateModel
import core.opnav as opnav

SQL_PREFIX = "sqlite:///"
sql_path = SQL_PREFIX + os.path.join("D:", "OpNav", "db", "satellite-db.sqlite")


def calculate_position_error(true_state, est_state):
    return math.sqrt(np.sum((true_state[:3] - est_state[:3]) ** 2))

def calculate_velocity_error(true_state, est_state):
    return math.sqrt( np.sum((true_state[3:6] - est_state[3:6])**2) )


################################################################
###################6 HOURS TRAJECTORY###########################
################################################################
# Testing Traj UKF on propagation data
# NOTE: trajectory is split into 6hr intervals due to impulses
# Original dataset is in minutes. timestep = 1
# will yield a dataset of size (part_end - part_start)
def sixhours_direct_test(visual_analysis, state_error, part_start, part_end, timestep):
    """
    Calls ukf directly without database overhead
    [part_start, part_end): start (inclusive) and end (exclusive) indices of trajectory
    [timestep]: interval in minutes
    """
    from tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
    d_camMeas, d_moonEph, d_sunEph, d_traj, totalIntegrationTime = get6HoursBatch(part_start, part_end, part_start,
                                                                                  timestep,
                                                                                  np.array([1, 1, 1, 1, 1, 1]),
                                                                                  np.array([1, 1, 1]),
                                                                                  np.array([1, 1, 1, 1]), 1, 1, 1,
                                                                                  att_meas=False)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))  # Initial Covariance Estimate of State
    state = (
        np.array([d_traj['x'][0], d_traj['y'][0], d_traj['z'][0], d_traj['vx'][0], d_traj['vy'][0], d_traj['vz'][0]],
                 dtype=float)).reshape(6, 1)
    last_estimate_dt_seconds = 200
    state[0:3] = state[0:3] - last_estimate_dt_seconds * state[3:6]
    R = np.diag(np.array(state_error, dtype=float))
    error = np.random.multivariate_normal(np.zeros((6,)), R).reshape(6, 1)
    state = state + error

    liveTraj = None
    if visual_analysis == "True":
        print("Visual Analysis on")
        liveTraj = LiveTrajectoryPlot()
    
    first = true
    dt = last_estimate_dt_seconds
    # print(int(trajTruthdf.shape[0]*1 - 1))
    for t in tqdm(range( len(d_traj['x']) ), desc ='Trajectory Completion'):
        # if t == 0: continue
        moonEph = (np.array(
            [d_moonEph['x'][t], d_moonEph['y'][t], d_moonEph['z'][t], d_moonEph['vx'][t], d_moonEph['vy'][t],
             d_moonEph['vz'][t]], dtype=float)).reshape(1, 6)
        sunEph = (np.array([d_sunEph['x'][t], d_sunEph['y'][t], d_sunEph['z'][t], d_sunEph['vx'][t], d_sunEph['vy'][t],
                            d_sunEph['vz'][t]], dtype=float)).reshape(1, 6)
        meas = (np.array(
            [d_camMeas['z1'][t], d_camMeas['z2'][t], d_camMeas['z3'][t], d_camMeas['z4'][t], d_camMeas['z5'][t],
             d_camMeas['z6'][t]], dtype=float)).reshape(6, 1)
        traj_out = runTrajUKF(
            EphemerisVector(moonEph[0, 0], moonEph[0, 1], moonEph[0, 2], moonEph[0, 3], moonEph[0, 4], moonEph[0, 5]),
            EphemerisVector(sunEph[0, 0], sunEph[0, 1], sunEph[0, 2], sunEph[0, 3], sunEph[0, 4], sunEph[0, 5]),
            CameraMeasurementVector(meas[0, 0], meas[1, 0], meas[2, 0], meas[3, 0], meas[4, 0], meas[5, 0]),
            TrajectoryStateVector.from_numpy_array(state),
            dt,
            CovarianceMatrix(matrix=P),
            MatlabTestCameraParameters,
            dynamicsOnly=False)  # used to have this, not sure why it's gone orientation=orientation
        state = traj_out.new_state.data
        P = traj_out.new_P.data
        K = traj_out.K
        # Per iteration error
        traj = (np.array(
            [d_traj['x'][t], d_traj['y'][t], d_traj['z'][t], d_traj['vx'][t], d_traj['vy'][t], d_traj['vz'][t]],
            dtype=float)).reshape(6, 1)
        traj = traj.flatten()
        fstate = state.flatten()
        posError = math.sqrt( np.sum((traj[:3] - fstate[:3])**2) )
        velError = math.sqrt( np.sum((traj[3:6] - fstate[3:6])**2) )
        # plot
        if liveTraj:
            liveTraj.updateEstimatedTraj(fstate[0], fstate[1], fstate[2])
            liveTraj.updateTrueTraj(traj[0], traj[1], traj[2])
            liveTraj.renderUKF(text="Iteration {} - Pos Error {} - Vel Error {}".format(t, round(posError), round(velError)))
        if first:
            dt=timestep*60

    if liveTraj:
        liveTraj.close()

    t = int((part_end - part_start)/timestep - 1)
    traj = (np.array([d_traj['x'][t],
                      d_traj['y'][t],
                      d_traj['z'][t],
                      d_traj['vx'][t],
                      d_traj['vy'][t],
                      d_traj['vz'][t]], dtype=float)).reshape(6, 1)
        
    traj = traj.flatten()
    state = state.flatten()
    print(f'true_state: {traj}')
    print(f'est_state: {state}')
    posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
    velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
    print(f'Position error: {posError}\nVelocity error: {velError}')
    assert posError <= POS_ERROR_6HOURS, 'Position error is too large'
    assert velError <= VEL_ERROR_6HOURS, 'Velocity error is too large'

def sixhours_db_test(mocker, state_error, part_start, part_end, timestep):
    """
    Calls ukf using the opnav pipeline, which uses databases
    [timestep]: interval in minutes
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
    d_camMeas, d_moonEph, d_sunEph, d_traj, totalIntegrationTime = get6HoursBatch(part_start, part_end, part_start,
                                                                                  timestep,
                                                                                  np.array([1, 1, 1, 1, 1, 1]),
                                                                                  np.array([1, 1, 1]),
                                                                                  np.array([1, 1, 1, 1]), 1, 1, 1,
                                                                                  att_meas=False)
    assert len(d_camMeas['z1']) == len(d_traj['x'])
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))  # Initial Covariance Estimate of State
    R = np.diag(np.array(state_error, dtype=float))
    # init state
    state = (
        np.array([d_traj['x'][0], d_traj['y'][0], d_traj['z'][0], d_traj['vx'][0], d_traj['vy'][0], d_traj['vz'][0]],
                 dtype=float)).reshape(6, 1)
    last_estimate_dt_seconds = 200
    state[0:3] = state[0:3] - last_estimate_dt_seconds * state[3:6]
    error = np.random.multivariate_normal(np.zeros((6,)), R).reshape(6, 1)
    state = state + error
    state = state.flatten()
    entries = [
        OpNavTrajectoryStateModel.from_tuples(
            position=(state[0], state[1], state[2]), 
            velocity=(state[3], state[4], state[5]), 
            P=TrajUKFConstants.P0.reshape(6,6), 
            time=time+timedelta(0,-last_estimate_dt_seconds)), # initial state used by propulsion step, output used by propagation step
        OpNavAttitudeStateModel.from_tuples(
            quat=(0., 0., 0., 0.), 
            rod_params=(0., 0., 0.), 
            biases=(0.,0.,0.,), 
            P=np.zeros((6,6)), 
            time=time), # dummy attitude states
    ]
    measurements = []
    timestamps = []
    for t in tqdm(range( len(d_traj['x']) ), desc ='Preparing test dbs'):
        new_time = time+timedelta(0,timestep*60*t)
        timestamps.append(new_time)
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
    progress_bar = tqdm(total = len(measurements), desc='Progress')

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
        progress_bar.update(1)
        return OPNAV_EXIT_STATUS.SUCCESS

    mocker.patch('core.opnav.__observe', side_effect=observe_mock)

    def process_propulsion_events_mock(*args, **kwargs):
        return OPNAV_EXIT_STATUS.SUCCESS

    mocker.patch('core.opnav.__process_propulsion_events', side_effect=process_propulsion_events_mock)

    def run_attitude_ukf_mock(*args, **kwargs):
        return AttitudeEstimateOutput(new_state=AttitudeStateVector(0,0,0,0,0,0),
                                                    new_P=CovarianceMatrix(matrix=np.zeros((6,6))), 
                                                    new_quat=QuaternionVector(0, 0, 0, 0))

    mocker.patch('core.attitude.runAttitudeUKF', side_effect=run_attitude_ukf_mock)

    # def run_trajectory_ukf_mock(*args, **kwargs):
    #     print("----------TrajUKF Mock----------")
    #     return TrajectoryEstimateOutput(new_state=TrajectoryStateVector(0,0,0,0,0,0), new_P=CovarianceMatrix(matrix=np.zeros((6,6))), K=Matrix6x6(matrix=np.zeros((6,6))))
    # mocker.patch('core.ukf.runTrajUKF', side_effect=run_trajectory_ukf_mock)
    
    temp_db_len = len(session.query(OpNavTrajectoryStateModel).all())
    assert temp_db_len == 1, f'Expected 1 trajectory state in db, got {temp_db_len}'
    # start opnav system
    status = opnav.start(sql_path=sql_path, num_runs=len(measurements), gyro_count=2, camera_params=MatlabTestCameraParameters)
    # verification
    entries = session.query(OpNavTrajectoryStateModel).all()
    # print(entries)
    expected_new_entries_len = len(d_traj['x'])
    assert len(
        entries) == expected_new_entries_len + 1, f'Expected {expected_new_entries_len + 1} entries in trajectory db, got {len(entries)}'
    # start() should succeed
    assert status == OPNAV_EXIT_STATUS.SUCCESS, f'Expected start() to succeed, got status {status}'
    # position and velocity errors should be within bounds
    # i = len(d_traj['x']) - 1
    # entry = entries[-1]
    # for i, timestamp in tqdm(enumerate(timestamps), desc=""):
    i = len(timestamps) - 1
    entry = session.query(OpNavTrajectoryStateModel).filter(OpNavTrajectoryStateModel.time_retrieved == timestamps[
        -1]).first()  # search for state estimate made at time of measurement
    true_state = np.array(
        [d_traj['x'][i], d_traj['y'][i], d_traj['z'][i], d_traj['vx'][i], d_traj['vy'][i], d_traj['vz'][i]],
        dtype=float).flatten()
    est_state = np.array(
        [entry.position_x, entry.position_y, entry.position_z, entry.velocity_x, entry.velocity_y, entry.velocity_z],
        dtype=float).flatten()
    posError = calculate_position_error(true_state, est_state)
    velError = calculate_velocity_error(true_state, est_state)
    print(f'true_state: {true_state}')
    print(f'est_state: {est_state}')
    print(f'Position error: {posError}\nVelocity error: {velError}')
    assert posError <= POS_ERROR_6HOURS, f'Position error is too large. Expected {POS_ERROR_6HOURS} Actual {posError}'
    assert velError <= VEL_ERROR_6HOURS, f'Velocity error is too large. Expected {VEL_ERROR_6HOURS} Actual {velError}'
    progress_bar.close()

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
    def setup_method(self,function):
        print(f'Running setup for {self.__class__.__name__}::{function.__name__}')

    def teardown_method(self,function):
        print(f'Running teardown for {self.__class__.__name__}::{function.__name__}')
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
    
    def test_interval1_db(self, mocker):
        sixhours_db_test(mocker, ZERO_STARTING_NOISE, 0, 360, 1.)

    def test_interval1_direct(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 0, 360, 1.)

    def test_interval2(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 361, 710, 1)

    def test_interval3(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 760, 1000, 1)
        
    def test_interval4(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 1100, 1400, 1)

    def test_interval5_db(self, mocker):
        sixhours_db_test(mocker, ZERO_STARTING_NOISE, 0, 360, 2.)

    def test_interval5_direct(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 0, 360, 2.)

    def test_interval6(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 361, 710, 2)

    def test_interval7(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 760, 1000, 2)
        
    def test_interval8(self, mocker):
        sixhours_direct_test(False, ZERO_STARTING_NOISE, 1100, 1400, 2)

"""
Assumes starting state provided by NASA is a bit noisy.
Each run is treated as a continuation of the previous state.
Tests:
- expected state is close to actual state
- 
"""
@pytest.mark.small_noise_test
class TestSmallStartingNoiseTimestep:
    def setup_method(self,function):
        print(f'Running setup for {self.__class__.__name__}::{function.__name__}')

    def teardown_method(self,function):
        print(f'Running teardown for {self.__class__.__name__}::{function.__name__}')
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

    def test_interval1_db(self, mocker):
        sixhours_db_test(mocker, SMALL_STARTING_NOISE, 0, 360, 1.)

    def test_interval1_direct(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 0, 360, 1.)

    def test_interval2(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 361, 710, 1)

    def test_interval3(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 760, 1000, 1)
        
    def test_interval4(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 1100, 1400, 1)

    def test_interval5_db(self, mocker):
        sixhours_db_test(mocker, SMALL_STARTING_NOISE, 0, 360, 2.)

    def test_interval5_direct(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 0, 360, 2.)

    def test_interval6(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 361, 710, 2)

    def test_interval7(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 760, 1000, 2)
        
    def test_interval8(self, mocker):
        sixhours_direct_test(False, SMALL_STARTING_NOISE, 1100, 1400, 2)

"""
Assumes starting state provided by NASA is very noisy.
Each run is treated as a continuation of the previous state.
Tests:
- expected state is close to actual state
- 
"""
@pytest.mark.large_noise_test
class TestLargeStartingNoiseTimestep:
    def setup_method(self,function):
        print(f'Running setup for {self.__class__.__name__}::{function.__name__}')

    def teardown_method(self,function):
        print(f'Running teardown for {self.__class__.__name__}::{function.__name__}')
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

    def test_interval1_db(self, mocker):
        sixhours_db_test(mocker, LARGE_STARTING_NOISE, 0, 360, 1.)

    def test_interval1_direct(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 0, 360, 1.)

    def test_interval2(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 361, 710, 1)

    def test_interval3(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 760, 1000, 1)
        
    def test_interval4(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 1100, 1400, 1) # TODO: Volatile test: depends on random starting noise

    def test_interval5_db(self, mocker):
        sixhours_db_test(mocker, LARGE_STARTING_NOISE, 0, 360, 2.) 

    def test_interval5_direct(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 0, 360, 2.)

    def test_interval6(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 361, 710, 2)

    def test_interval7(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 760, 1050, 2)
        
    def test_interval8(self, mocker):
        sixhours_direct_test(False, LARGE_STARTING_NOISE, 1500, 1800, 2) # TODO: Volatile test: depends on random starting noise
