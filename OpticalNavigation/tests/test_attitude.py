import pytest

from core.attitude import runAttitudeUKFWithKick

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
from scipy.stats import norm
from numpy import random
from copy import deepcopy
from scipy.interpolate import InterpolatedUnivariateSpline
from tqdm import tqdm

from tests.gen_opnav_data import get6HoursBatch, plotResults

def test_attitude_6_hours(visual_analysis):
    secPerMin = 60
    minPerHour = 60
    cameraDt = 1
    omegaInit = [0., 0.001, 2., 0., 0., 0.]
    biasInit=[0., 0., 0.]
    quat = np.array([[np.random.randn(), np.random.randn(), np.random.randn(), np.random.randn()]]).T
    gyroSampleCount = 5
    coldGasKickTime = 100
    
    gyro_noise_sigma = 1.e-7
    gyro_sigma = 1.e-10
    gyro_t = 1.0 / gyroSampleCount
    meas_sigma = 8.7e-4
    # timeNoiseSigma=2
    
    startTime = 0
    # numberOfMeasForTest = 100
    endTime = 700
    
    q1, q2, q3, q4, omegax, omegay, omegaz, biasx, biasy, biasz, d_camMeas, d_moonEph, d_sunEph, d_traj, earthVectors, moonVectors, sunVectors, totalIntegrationTime = \
        get6HoursBatch(startTime, endTime, coldGasKickTime, cameraDt, omegaInit, biasInit, quat, gyroSampleCount, gyro_sigma, gyro_noise_sigma, att_meas=True)
    numberOfMeasForTest = len(d_camMeas['z1'])
    
    P0 = np.array([[1.e-1, 0., 0., 0., 0., 0.],
                  [0., 1.e-1, 0., 0., 0., 0.],
                  [0., 0., 1.e-1, 0., 0., 0.],
                  [0., 0., 0., 9.7e-10, 0., 0.],
                  [0., 0., 0., 0., 9.7e-10, 0.],
                  [0., 0., 0., 0., 0., 9.7e-10]]) * 10.

    Q = np.array([[gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_t**2., 0., 0., 0., 0., 0.],
                    [0., gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_t**2., 0., 0., 0., 0.],
                    [0., 0., gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_t**2., 0., 0., 0.],
                    [0., 0., 0., gyro_sigma**2., 0., 0.],
                    [0., 0., 0., 0., gyro_sigma**2., 0.],
                    [0., 0., 0., 0., 0., gyro_sigma**2.]]) * .5*gyro_t

    R = np.eye(9) * meas_sigma**2.

    x0 = np.array([[0., 0., 0., 0., 0., 0.]]).T

    # Create dummy position UKF outputs
    estimatedSatStates = np.zeros((len(d_camMeas['time']*gyroSampleCount), 3))
    moonEph = np.zeros((len(d_camMeas['time']*gyroSampleCount), 3))
    sunEph = np.zeros((len(d_camMeas['time']*gyroSampleCount), 3))
    omegaMeasurements = np.zeros((len(d_camMeas['time']*gyroSampleCount), 3))
    # assuming biases can be measured. 
    biasMeasurements = np.zeros((len(d_camMeas['time']*gyroSampleCount), 3))
    timeline = [0.]*(len(d_camMeas['time']*gyroSampleCount))
    count = 0
    for i in tqdm(range(numberOfMeasForTest), desc='Trajectory Estimates'):
        # Sat
        pos = np.array([d_traj['x'][i], d_traj['y'][i], d_traj['z'][i]]) + np.random.normal(scale=100, size=3)
        vel = np.array([d_traj['vx'][i], d_traj['vy'][i], d_traj['vz'][i]])
        start_t = d_traj['time'][i]
        times = start_t + gyro_t * np.arange(0, gyroSampleCount)
        tempCount = count
        for i_,t in enumerate(times):
            pos = pos + t * vel # propogate position estimate to gyro measurement time
            estimatedSatStates[tempCount,:] = pos
            tempCount += 1
        # Moon
        pos = np.array([d_moonEph['x'][i], d_moonEph['y'][i], d_moonEph['z'][i]]) + np.random.normal(scale=100, size=3)
        vel = np.array([d_moonEph['vx'][i], d_moonEph['vy'][i], d_moonEph['vz'][i]])
        tempCount = count
        for i_,t in enumerate(times):
            pos = pos + t * vel # propogate position estimate to gyro measurement time
            moonEph[tempCount,:] = pos
            tempCount += 1
        # Sun
        pos = np.array([d_sunEph['x'][i], d_sunEph['y'][i], d_sunEph['z'][i]]) + np.random.normal(scale=100, size=3)
        vel = np.array([d_sunEph['vx'][i], d_sunEph['vy'][i], d_sunEph['vz'][i]])
        tempCount = count
        for i_,t in enumerate(times):
            pos = pos + t * vel # propogate position estimate to gyro measurement time
            sunEph[tempCount,:] = pos
            tempCount += 1

        # omegas and biases
        tempCount = count
        for i_,t in enumerate(times):
            omegaMeasurements[tempCount, :] = np.array([float(omegax(t)), float(omegay(t)), float(omegaz(t))])
            biasMeasurements[tempCount, :] = np.array([float(biasx(t)), float(biasy(t)), float(biasz(t))])
            timeline[tempCount] = t
            tempCount += 1

        count = tempCount

    print("START")
    results = runAttitudeUKFWithKick(cameraDt, (gyro_sigma, gyro_t, Q, R), P0, x0, quat, omegaMeasurements, biasMeasurements, estimatedSatStates, moonEph, sunEph, timeline)
    print("END")

    plotResults(results, q1, q2, q3, q4, biasx, biasy, biasz, timeline)