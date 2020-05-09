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

from gen_opnav_data import get6HoursBatch, plotResults
from core.const import P0, R


def test_attitude_6_hours(visual_analysis):
    secPerMin = 60
    minPerHour = 60
    cameraDt = 6 * secPerMin * minPerHour
    omegaInit = [0., 0.001, 2., 0., 0., 0.]
    biasInit=[0., 0., 0.]
    quat = np.array([[np.random.randn(), np.random.randn(), np.random.randn(), np.random.randn()]]).T
    gyroSampleCount = 4
    coldGasKickTime = 100
    
    gyro_noise_sigma = 1.e-4
    gyro_sigma = 1.e-10
    gyro_t = 1.0 / gyroSampleCount
    meas_sigma = 8.7e-4
    timeNoiseSigma=2
    
    q1, q2, q3, q4, omegax, omegay, omegaz, biasx, biasy, biasz, d_camMeas, d_moonEph, d_sunEph, d_traj, earthVectors, moonVectors, sunVectors = \
        get6HoursBatch(0, None, coldGasKickTime, cameraDt, omegaInit, biasInit, quat, gyroSampleCount, timeNoiseSigma, gyro_sigma, gyro_noise_sigma)
    
    x0 = np.array([[0., 0., 0., 0., 0., 0.]]).T
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

    results = runAttitudeUKFWithKick(cameraDt, omegax, omegay, omegaz, biasx, biasy, biasz, earthVectors, moonVectors, sunVectors, d_camMeas, x0, quat, P0, coldGasKickTime)
    
    plotResults(results, q1, q2, q3, q4, biasx, biasy, biasz)