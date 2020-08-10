import pytest
import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm

from core.opnav import process
from tests.const import CesiumTestCameraParameters

# from core.ukf import runPosVelUKF
# from tests.const import POS_ERROR, VEL_ERROR
# from tests.const import ZERO_STARTING_NOISE, SMALL_STARTING_NOISE, LARGE_STARTING_NOISE
# from tests.const import MatlabTestCameraParameters
# from tests.animations import LiveTrajectoryPlot


def test_RandomData():
    # M M M F F
    batch = [
        # Attitude adjustment begins
        {
            'cam_meas': np.array([1366.930869,37.69615365,1354.540882,152,6,34]).reshape(6,1),
            'cam_dt': 60,   # seconds since last cam_meas.
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
    traj_P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
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

    process(batch, initTrajState, traj_P, cameraParams, att_P, initAttitudeState, initQuaternionState, gyroVars)






# def test_cislunarFullTrajectoryWithManeuvers_zero_starting_noise(visual_analysis):
#     """
#     Assumes starting state provided by NASA is very noisy.
#     Each run is treated as a continuation of the previous state.
#     Tests:
#     - expected state is close to actual state
#     - 
#     """
#     cislunarFullTrajectoryWithManeuvers(visual_analysis, ZERO_STARTING_NOISE, 0, 360)