from OpticalNavigation.core.const import CameraMeasurementVector
import pytest
import pandas as pd
import numpy as np
import math
import os
import re
from tqdm import tqdm
from datetime import datetime, timedelta
from time import sleep
from unittest.mock import patch

import OpticalNavigation.core.opnav as opnav
import OpticalNavigation.core.preprocess as preprocess
import OpticalNavigation.core.sense as sense
import OpticalNavigation.core.const as opnav_constants
from OpticalNavigation.tests.const import CesiumTestCameraParameters

from utils.db import create_sensor_tables_from_path, OpNavTrajectoryStateModel, OpNavAttitudeStateModel
from utils.db import OpNavEphemerisModel, OpNavCameraMeasurementModel, OpNavPropulsionModel, OpNavGyroMeasurementModel, RebootsModel
from utils.constants import DB_FILE
import utils.parameters as params

# from core.ukf import runTrajUKF
# from tests.const import POS_ERROR, VEL_ERROR
# from tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
# from tests.const import MatlabTestCameraParameters
# from tests.animations import LiveTrajectoryPlot

SQL_PREFIX = "sqlite:///"
sql_path = SQL_PREFIX + os.path.join("/home", "stephen_z", "Desktop", "test.sqlite")
#sql_path = DB_FILE

def setup_function(function):
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

    new_bootup = RebootsModel(is_bootup=False, reboot_at=datetime(2020, 7, 28, 22, 8, 3))
    session.add(new_bootup)
    session.commit()

def teardown_function(function):
    print("teardown...")

def test_start(mocker):
    # Add data to test database
    create_session = create_sensor_tables_from_path(sql_path)
    session = create_session()
    # entries = session.query(OpNavPropulsionModel).all()
    time = datetime(2020, 7, 28, 22, 8, 3)
    entries = [
        #OpNavPropulsionModel.from_tuples(
        #0.0001, time_start=time+timedelta(0,-110), time_end=time+timedelta(0,-100)), # 10 second propulsion that occured 100 seconds ago
        OpNavTrajectoryStateModel.from_tuples(
        (0.,2.,3.), (2.,2.,3.), opnav_constants.TrajUKFConstants.P0.reshape(6,6), time=time+timedelta(0,-200)), # initial state used by propulsion step, output used by propagation step
        OpNavAttitudeStateModel.from_tuples(
            (0., 0., 0., 1.), (0., 1., 0.), (0.,2.,0.,), opnav_constants.AttitudeUKFConstants.P0.reshape(6,6), time=time+timedelta(0,-200)),

        #OpNavEphemerisModel.from_tuples(
        #    sun_eph=(1111., 1111., 1111., 124., 24., 4.), moon_eph=(4244., 42424., 4244., 42., 42., 42.), time=time+timedelta(0,20)),
        #OpNavEphemerisModel.from_tuples(
         #   sun_eph=(2222., 2222., 2222., 124., 24., 4.), moon_eph=(4244., 42424., 4244., 42., 42., 42.), time=time+timedelta(0,-3)),
        #OpNavEphemerisModel.from_tuples(
          #  sun_eph=(3333., 3333., 3333., 124., 24., 4.), moon_eph=(4244., 42424., 4244., 42., 42., 42.), time=time+timedelta(1,0)),

        #OpNavCameraMeasurementModel.from_tuples(
        #    measurement=(0.2, 0.1, 0.2, 0.1, 0.11, 0.22), time=time), # camera measurement taken now, uses state

        #OpNavGyroMeasurementModel.from_tuples(
        #    measurement=(0., 2., 1.), time=time+timedelta(0,-50)), # Not picked because gyro was read before this run's corresponding cam meas
        #OpNavGyroMeasurementModel.from_tuples(
        #    measurement=(2., 1., 1.), time=time+timedelta(0,0.1)),
        #OpNavGyroMeasurementModel.from_tuples(
        #    measurement=(2., 1., 1.), time=time+timedelta(0,0.21234)),
        #OpNavGyroMeasurementModel.from_tuples(
        #    measurement=(2., 1., 1.), time=time+timedelta(0,0.31244)),
    ]
    session.bulk_save_objects(entries)
    session.commit()
    # mocks
    # do not test observe()
    def observe_mock(session, gyro_count):
        return opnav_constants.OPNAV_EXIT_STATUS.SUCCESS
    #mocker.patch('OpticalNavigation.core.opnav.__observe', side_effect=observe_mock)

    def select_camera_mock(id):
        print("select_mock")
        return

    #Don't use select_camera_mock for software demo
    #mocker.patch('OpticalNavigation.core.opnav.select_camera', side_effect=select_camera_mock)

    idx = [0]
    timestamps = [981750, 981750, 2028950, 2028950, 3010700, 3010700]
    videos = ['/home/stephen_z/Desktop/test-videos/cam1_expLow.mjpeg', '/home/stephen_z/Desktop/test-videos/cam1_expHigh.mjpeg',
              '/home/stephen_z/Desktop/test-videos/cam2_expLow.mjpeg', '/home/stephen_z/Desktop/test-videos/cam2_expHigh.mjpeg',
              '/home/stephen_z/Desktop/test-videos/cam3_expLow.mjpeg', '/home/stephen_z/Desktop/test-videos/cam3_expHigh.mjpeg']

    def record_video_mock(filename, framerate, recTime, exposure):
        print("video_mock")
        time = timestamps[idx[0]]
        filename = videos[idx[0]]
        #idx[0] += 1
        return (filename, 0, time)

    #Don't use record_video mock for software demo
    #mocker.patch('OpticalNavigation.core.opnav.record_video', side_effect=record_video_mock)

    def record_gyro_mock(count):
        print("gyro_mock")
        return np.array([[0, 5, 0]])
    mocker.patch('OpticalNavigation.core.opnav.record_gyro', side_effect=record_gyro_mock)


    path = '../../../../Desktop/cislunar_case1c/'
    idx = [0]
    names = ['cam1_expLow_f', 'cam1_expHigh_f',
              'cam2_expLow_f', 'cam2_expHigh_f',
              'cam3_expLow_f', 'cam3_expHigh_f']
    #timestamps = [0, 0, 2094400, 2094400, 4188800, 4188800]
    timestamps = [0, 6283200, 2094400, 2094400 + 6283200, 4188800, 4188800 + 6283200]
    interval = [65450]
    camera_rec_params = opnav_constants.CameraRecordingParameters(params.CAMERA_FPS, params.CAMERA_RECORDING_TIME, params.CAMERA_LOW_EXPOSURE, params.CAMERA_HIGH_EXPOSURE)
    def extract_frames_mock(vid_dir, frameDiff, endTimestamp, cameraRecParams):
        frames = []
        for i in range(0,20):
            frames.append(path + names[idx[0]] + str(i) + "_t" + str(int(round((timestamps[idx[0]] + interval[0]*i)/1000-0.01))) + ".jpg")
            #print((timestamps[idx[0]] + interval[0]*i)/1000)
            #print(frames[-1])
        print("frames_mock")
        idx[0] += 1
        return frames
    #mocker.patch('OpticalNavigation.core.opnav.extract_frames', side_effect=extract_frames_mock)


    def __get_elapsed_time_mock(bestTuple, timeDeltaAvgs, observeStart):
        print("get_elapsed_time_mock")
        observeStart = datetime.utcfromtimestamp(observeStart * 10**-6)
        lastReboot = datetime(2020, 7, 28, 22, 8, 3)
        timestamp = int(re.search(r'[t](\d+)', bestTuple[0]).group(1)) * 1000 # factor if 1000 ONLY for case1c
        dateTime = lastReboot + timedelta(microseconds=timestamp)
        timeElapsed = (dateTime - observeStart).total_seconds()
        return timeElapsed

    mocker.patch('OpticalNavigation.core.opnav.__get_elapsed_time', side_effect=__get_elapsed_time_mock)

    # start opnav system
    opnav.start(sql_path=sql_path, num_runs=1, gyro_count=2)
    # TODO: verification
    # - there should be a new entry in trajectory db with time_retrieved set to propulsion end time (for each propulsion entry)
    # - propulsion db should be empty
    # - start() should succeed
    # - process should have picked closest ephemeris to propulsion start time
    # - picks the latest states and cam meas
    # - picks latest gyro meas that came after the cam meas (# gyro meas picked for processing is not dependent on gyro_count)
    # - order in which data arrive in inital db matters. Opnav assumes state db's have an initial estimate with the oldest time of
    #   all entries in all dbs (except ephemerides). The second oldest should be propulsion and the newest should be measurements.
    # - during propulsion step, traj state is out of sync with att state (it shouldn't matter because they are independent of one another)


def test_RandomData():
    # M M M F F
    batch = [
        # Attitude adjustment begins
        {
            # [z1, z2, z3, z4, z5, z6]
            # z1 = pixel distance E-M
            # z2 = pixel distance E-S
            # z3 = pixel distance S-M
            # z4, z5, z6 = pixel widths of E, M, S
            'cam_meas': np.array([1366.930869,37.69615365,1354.540882,152,6,34]).reshape(6,1),
            'cam_dt': 60,   # seconds since last cam_meas.
            'ephemeris': {
                # x pos (km), y pos (km), z pos (km), vx (km/s), vy (km/s), vz (km/s) [J2000 ECI]
                'moon': np.array([-343713.6722,-145772.8313,-27946.53566,0.366284,-0.914878,-0.37493]).reshape(1,6),
                'sun': np.array([-143591482.3,-41349754.27,2767.572392,8.732749154,-28.5257364,0.001459914]).reshape(1,6)
            },
            'gyro_meas':[
                # Assuming 4 gyro measurements
                {
                    'dt': 0.01, # seconds elpased between last image taken and gyro sampled for the 1st time
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3) # Assume these are random (there is no way of recording biases)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
            ],
            'main_thrust':{
                'fire':False,
                'thrust_mag':0
            }
        },
        # Attitude adjust ends. Getting ready to fire main thruster
        {
            'cam_meas': np.array([1366.930869,37.69615365,1354.540882,152,6,34]).reshape(6,1),
            'cam_dt': 0.1,   # seconds since last cam_meas after 4th gyro sample (assuming hardware/IO delays)
            'ephemeris': {
                'moon': np.array([-343713.6722,-145772.8313,-27946.53566,0.366284,-0.914878,-0.37493]).reshape(1,6),
                'sun': np.array([-143591482.3,-41349754.27,2767.572392,8.732749154,-28.5257364,0.001459914]).reshape(1,6)
            },
            'gyro_meas':[
                # Assuming 4 gyro measurements
                {
                    'dt': 0.01, # seconds elpased between last image taken and gyro sampled for the 1st time
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3) # Assume these are random (there is no way of recording biases)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
            ],
            'main_thrust':{
                'fire':False,
                'thrust_mag':0
            }
        },
        # This will be the last opnav run. After this, propulsion takes control of flight software...
        {
            'cam_meas': np.array([1366.930869,37.69615365,1354.540882,152,6,34]).reshape(6,1),
            'cam_dt': 0.1,  
            'ephemeris': {
                'moon': np.array([-343713.6722,-145772.8313,-27946.53566,0.366284,-0.914878,-0.37493]).reshape(1,6),
                'sun': np.array([-143591482.3,-41349754.27,2767.572392,8.732749154,-28.5257364,0.001459914]).reshape(1,6)
            },
            'gyro_meas':[
                # Assuming 4 gyro measurements
                {
                    'dt': 0.01, # seconds elpased between last image taken and gyro sampled for the 1st time
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3) # Assume these are random (there is no way of recording biases)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
            ],
            'main_thrust':{
                'fire':False,
                'thrust_mag':0
            }
        },
        # Main-thruster firing for some time. Next run.
        {
            'cam_meas': np.array([1366.930869,37.69615365,1354.540882,152,6,34]).reshape(6,1),
            'cam_dt': 5,   # Fire for 5 seconds. 
            'ephemeris': {
                'moon': np.array([-343713.6722,-145772.8313,-27946.53566,0.366284,-0.914878,-0.37493]).reshape(1,6),
                'sun': np.array([-143591482.3,-41349754.27,2767.572392,8.732749154,-28.5257364,0.001459914]).reshape(1,6)
            },
            'gyro_meas':[
                # Assuming 4 gyro measurements
                {
                    'dt': 0.01, # seconds elpased between last image taken and gyro sampled for the 1st time
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3) # Assume these are random (there is no way of recording biases)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
            ],
            'main_thrust':{ # Thrust fire dt is assumed to be the same as camera measurement dt
                'fire':True,
                'thrust_mag':0.0005 # km/sec^2
            }
        },
        # Another burn...
        {
            'cam_meas': np.array([1366.930869,37.69615365,1354.540882,152,6,34]).reshape(6,1),
            'cam_dt': 5,   # Fire for 5 seconds
            'ephemeris': {
                'moon': np.array([-343713.6722,-145772.8313,-27946.53566,0.366284,-0.914878,-0.37493]).reshape(1,6),
                'sun': np.array([-143591482.3,-41349754.27,2767.572392,8.732749154,-28.5257364,0.001459914]).reshape(1,6)
            },
            'gyro_meas':[
                # Assuming 4 gyro measurements
                {
                    'dt': 0.01, # seconds elpased between last image taken and gyro sampled for the 1st time
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3) # Assume these are random (there is no way of recording biases)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
                {
                    'dt': 0.01, # seconds since previous gyro measurement
                    'omega': np.random.rand(1,3),
                    'bias': np.random.rand(1,3)
                },
            ],
            'main_thrust':{
                'fire':True,
                'thrust_mag':0.001
            }
        },
    ]

    # 
    initTrajState = np.random.rand(6,1)
    #
    traj_P = opnav_constants.TrajUKFConstants.P0
    #
    cameraParams = CesiumTestCameraParameters
    gyro_sigma = 1.e-10
    gyro_sample_rate = 0.01
    gyro_noise_sigma = 1.e-7
    meas_sigma = 8.7e-4
    #
    att_P = np.array([[1.e-1, 0., 0., 0., 0., 0.],
                  [0., 1.e-1, 0., 0., 0., 0.],
                  [0., 0., 1.e-1, 0., 0., 0.],
                  [0., 0., 0., 9.7e-10, 0., 0.],
                  [0., 0., 0., 0., 9.7e-10, 0.],
                  [0., 0., 0., 0., 0., 9.7e-10]]) * 10.

    Q = np.array([[gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_sample_rate**2., 0., 0., 0., 0., 0.],
                    [0., gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_sample_rate**2., 0., 0., 0., 0.],
                    [0., 0., gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_sample_rate**2., 0., 0., 0.],
                    [0., 0., 0., gyro_sigma**2., 0., 0.],
                    [0., 0., 0., 0., gyro_sigma**2., 0.],
                    [0., 0., 0., 0., 0., gyro_sigma**2.]]) * .5*gyro_sample_rate

    R = np.eye(9) * meas_sigma**2.
    #
    initAttitudeState = np.array([[0., 0., 0., 0., 0., 0.]]).T
    #
    initQuaternionState = np.array([[np.random.randn(), np.random.randn(), np.random.randn(), np.random.randn()]]).T
    #
    gyroVars = (gyro_sigma, gyro_sample_rate, Q, R)

    opnav.process(batch, initTrajState, traj_P, cameraParams, att_P, initAttitudeState, initQuaternionState, gyroVars)





# def test_cislunarFullTrajectoryWithManeuvers_zero_starting_noise(visual_analysis):
#     """
#     Assumes starting state provided by NASA is very noisy.
#     Each run is treated as a continuation of the previous state.
#     Tests:
#     - expected state is close to actual state
#     - 
#     """
#     cislunarFullTrajectoryWithManeuvers(visual_analysis, ZERO_STARTING_NOISE, 0, 360)
