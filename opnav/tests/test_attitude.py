from typing import List
from core.const import (
    AttitudeEstimateOutput,
    AttitudeStateVector,
    CovarianceMatrix,
    GyroMeasurementVector,
    GyroVars,
    QuaternionVector,
)
import pytest

from core.attitude import runAttitudeUKF
from simulations.animations import LiveMultipleAttitudePlot, Live2DPlot
from simulations.sim import isOrthogonal
from core.ukf import __attitudeMatrix

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
from scipy.stats import norm
from numpy import random
from copy import deepcopy
from scipy.interpolate import InterpolatedUnivariateSpline
from tqdm import tqdm

import tests.gen_opnav_data as generator


def test_attitude_6_hours():
    secPerMin = 60
    minPerHour = 60
    cameraDt = 1
    omegaInit = [0.0, 0.001, 6.0, 0.0, 0.0, 0.0]
    biasInit = [0.0, 0.0, 0.0]
    quat = np.array(
        [[np.random.randn(), np.random.randn(), np.random.randn(), np.random.randn()]]
    ).T
    gyroSampleCount = 4
    coldGasKickTime = 200

    gyro_noise_sigma = 1.0e-7
    gyro_sigma = 1.0e-10
    gyro_t = 1.0 / gyroSampleCount
    meas_sigma = 8.7e-4
    # timeNoiseSigma=2

    startTime = 0
    # numberOfMeasForTest = 100
    endTime = 700

    (
        q1,
        q2,
        q3,
        q4,
        omegax,
        omegay,
        omegaz,
        biasx,
        biasy,
        biasz,
        d_camMeas,
        d_moonEph,
        d_sunEph,
        d_traj,
        earthVectors,
        moonVectors,
        sunVectors,
        totalIntegrationTime,
    ) = generator.get6HoursBatch(
        startTime,
        endTime,
        coldGasKickTime,
        cameraDt,
        omegaInit,
        biasInit,
        quat,
        gyroSampleCount,
        gyro_sigma,
        gyro_noise_sigma,
        att_meas=True,
    )
    numberOfMeasForTest = len(d_camMeas["z1"])

    P0 = (
        np.array(
            [
                [1.0e-1, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0e-1, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0e-1, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 9.7e-10, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 9.7e-10, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 9.7e-10],
            ]
        )
        * 10.0
    )

    Q = (
        np.array(
            [
                [
                    gyro_noise_sigma ** 2.0
                    - (1.0 / 6.0) * gyro_sigma ** 2.0 * gyro_t ** 2.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    gyro_noise_sigma ** 2.0
                    - (1.0 / 6.0) * gyro_sigma ** 2.0 * gyro_t ** 2.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [
                    0.0,
                    0.0,
                    gyro_noise_sigma ** 2.0
                    - (1.0 / 6.0) * gyro_sigma ** 2.0 * gyro_t ** 2.0,
                    0.0,
                    0.0,
                    0.0,
                ],
                [0.0, 0.0, 0.0, gyro_sigma ** 2.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, gyro_sigma ** 2.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, gyro_sigma ** 2.0],
            ]
        )
        * 0.5
        * gyro_t
    )

    R = np.eye(9) * meas_sigma ** 2.0

    x0 = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

    gyroVars = GyroVars()
    gyroVars.gyro_noise_sigma = gyro_noise_sigma
    gyroVars.gyro_sample_rate = gyro_t
    gyroVars.gyro_sigma = gyro_sigma
    gyroVars.meas_sigma = meas_sigma

    # Create dummy position UKF outputs
    estimatedSatStates = np.zeros((len(d_camMeas["time"] * gyroSampleCount), 3))
    moonEph = np.zeros((len(d_camMeas["time"] * gyroSampleCount), 3))
    sunEph = np.zeros((len(d_camMeas["time"] * gyroSampleCount), 3))
    omegaMeasurements = np.zeros((len(d_camMeas["time"] * gyroSampleCount), 3))
    # assuming biases can be measured.
    # biasMeasurements = np.zeros((len(d_camMeas['time']*gyroSampleCount), 3))
    count = 0
    results = []
    resultsTimeline = []
    # For each camera measurement
    for i in tqdm(range(numberOfMeasForTest), desc="Trajectory Estimates"):
        # Sat
        pos = np.array(
            [d_traj["x"][i], d_traj["y"][i], d_traj["z"][i]]
        ) + np.random.normal(scale=0, size=3)
        vel = np.array([d_traj["vx"][i], d_traj["vy"][i], d_traj["vz"][i]])
        start_t = d_traj["time"][i]
        times = start_t + gyro_t * np.arange(0, gyroSampleCount)
        resultsTimeline.append(times[-1])  # time corresponding to new estimate
        tempCount = count
        for i_, t in enumerate(times):
            pos = pos + t * vel  # propogate position estimate to gyro measurement time
            estimatedSatStates[tempCount, :] = pos
            tempCount += 1
        # Moon
        pos = np.array(
            [d_moonEph["x"][i], d_moonEph["y"][i], d_moonEph["z"][i]]
        ) + np.random.normal(scale=100, size=3)
        vel = np.array([d_moonEph["vx"][i], d_moonEph["vy"][i], d_moonEph["vz"][i]])
        tempCount = count
        for i_, t in enumerate(times):
            pos = pos + t * vel  # propogate position estimate to gyro measurement time
            moonEph[tempCount, :] = pos
            tempCount += 1
        # Sun
        pos = np.array(
            [d_sunEph["x"][i], d_sunEph["y"][i], d_sunEph["z"][i]]
        ) + np.random.normal(scale=100, size=3)
        vel = np.array([d_sunEph["vx"][i], d_sunEph["vy"][i], d_sunEph["vz"][i]])
        tempCount = count
        for i_, t in enumerate(times):
            pos = pos + t * vel  # propogate position estimate to gyro measurement time
            sunEph[tempCount, :] = pos
            tempCount += 1

        # omegas and biases
        tempCount = count
        omegasMeasurementVectors: List[GyroMeasurementVector] = []
        for i_, t in enumerate(times):
            omegaMeasurements[tempCount, :] = np.array(
                [float(omegax(t)), float(omegay(t)), float(omegaz(t))]
            )
            # biasMeasurements[tempCount, :] = np.array([float(x0[3]), float(x0[4]), float(x0[5])])
            omegasMeasurementVectors.append(
                GyroMeasurementVector(
                    omega_x=float(omegax(t)),
                    omega_y=float(omegay(t)),
                    omega_z=float(omegaz(t)),
                )
            )
            tempCount += 1

        count = tempCount

        timeline = []
        for i, t in enumerate(times):
            if i == 0:
                timeline.append(0)
            else:
                timeline.append(t - timeline[-1])
        timeline[0] = sum(timeline) / (gyroSampleCount - 1.0)
        # run ukf
        gyroVarsTup = (
            gyroVars.gyro_sigma,
            gyroVars.gyro_sample_rate,
            gyroVars.get_Q_matrix(),
            gyroVars.get_R_matrix(),
        )

        estimate: AttitudeEstimateOutput = runAttitudeUKF(
            cameraDt,
            gyroVarsTup,
            CovarianceMatrix(matrix=P0),
            AttitudeStateVector.from_numpy_array(state=x0),
            QuaternionVector.from_numpy_array(quat=quat),
            omegasMeasurementVectors,
            estimatedSatStates,
            moonEph,
            sunEph,
            timeline,
        )
        results.append(estimate)
        P0 = estimate.new_P.data
        x0 = estimate.new_state.data
        quat = estimate.new_quat.data

    print(resultsTimeline)
    print(len(results), len(resultsTimeline))
    generator.plotResults(results, q1, q2, q3, q4, biasx, biasy, biasz, resultsTimeline)

    # # render data
    # liveAtt = LiveMultipleAttitudePlot(bounds=[-1, 1, -1, 1, -1, 1])
    # liveOmegas = Live2DPlot()

    # arrow1 = liveAtt.addArrowFromOrigin(color='r', label='X')
    # arrow2 = liveAtt.addArrowFromOrigin(color='g', label='Y')
    # arrow3 = liveAtt.addArrowFromOrigin(color='b', label='Z')

    # omx_id = liveOmegas.addGraph(color='r', label='omega x', alpha=0.5, ls='-', traceLim=50)
    # omy_id = liveOmegas.addGraph(color='g', label='omega y', alpha=0.5, ls='-', traceLim=50)
    # omz_id = liveOmegas.addGraph(color='b', label='omega z', alpha=0.5, ls='-', traceLim=50)
