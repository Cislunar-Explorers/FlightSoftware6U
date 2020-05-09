"""
Processing ephemeris tables and converting them into a usuable format.
"""

import pytest
import pandas as pd
import np as np
import math
import os
from tqdm import tqdm
from scipy.interpolate import InterpolatedUnivariateSpline
from scipy import integrate
from numpy.linalg import pinv
import numpy as np
import matplotlib.pyplot as plt

from animations import LiveTrajectoryPlot

from core.attitude import crs, meas_model, quaternionComposition, quaternionInv

from tests.const import TEST_6HOURS_meas, TEST_6HOURS_moonEph, TEST_6HOURS_sunEph, TEST_6HOURS_traj
from tests.const import TEST_C1_DISCRETIZED_meas, TEST_C1_DISCRETIZED_moonEph, TEST_C1_DISCRETIZED_sunEph, TEST_C1_DISCRETIZED_traj, TEST_C1_DISCRETIZED_matlab

from tests.const import SPACECRAFT_I_B, DAMPER_C, DAMPER_I_D, INTEGRATION_TIMESTEP

from core.const import _a, _f

"""
Generate Synthetic Data for Attitude Estimation
"""
def propagateSpacecraft(X, t, kickTime):
    """
    Integration step at time [t] with cold-gas thruster fired at [kickTime].
    """
    # print(X)
    omegasc1, omegasc2, omegasc3 = X[0], X[1], X[2]
    omegad1, omegad2, omegad3 = X[3], X[4], X[5]
    omega_sc = np.array([[omegasc1, omegasc2, omegasc3]]).T
    omega_d  = np.array([[omegad1, omegad2, omegad3]]).T
    
    inner_sc = (np.dot(crs(omega_sc), np.dot(SPACECRAFT_I_B, omega_sc)) - DAMPER_C*omega_d +
                    int(0.5*(np.sign(t - kickTime)+1))*np.array([[.1, 0., 0.]]).T -
                    int(0.5*(np.sign(t - (kickTime + 2))+1))*np.array([[.1, 0., 0.]]).T)
    omega_sc_dot = (np.dot(-1.*pinv(SPACECRAFT_I_B), inner_sc))
    
    right_d = np.dot(crs(omega_sc), np.dot(DAMPER_I_D, omega_d + omega_sc)) + DAMPER_C*omega_d
    inner_d = np.dot(DAMPER_I_D, omega_sc_dot) + right_d
    omega_d_dot = np.dot(-1.*pinv(DAMPER_I_D), inner_d)
    
    derivs = [omega_sc_dot[0][0], omega_sc_dot[1][0], omega_sc_dot[2][0],
              omega_d_dot[0][0], omega_d_dot[1][0], omega_d_dot[2][0]]
    
    return derivs

def goSpacecraft(X, tstop, delta_t, kickTime):
    """
    Integration for angular momentum and angular velocity
    """
    a_t = np.arange(0, tstop, delta_t)
    asol = integrate.odeint(propagateSpacecraft, X, a_t, args=(kickTime,))
    print("integration time steps {}, asol {}".format(len(a_t), len(asol)))
    hx, hy, hz = [], [], []
    htotx, htoty, htotz = [], [], []
    omegasx, omegasy, omegasz = [], [], []
    omegadx, omegady, omegadz = [], [], []
    h_norm_spacecraft = []
    hnorm = []
    for i in asol:
        omega_spacecraft = np.array([[i[0], i[1], i[2]]]).T
        omega_damper = np.array([[i[3], i[4], i[5]]]).T
        h = np.dot(SPACECRAFT_I_B, omega_spacecraft)
        h_norm_spacecraft.extend([np.linalg.norm(h)])
        hdamp = np.dot(DAMPER_I_D, omega_damper + omega_spacecraft)
        htot = h + hdamp
        hnorm.extend([np.linalg.norm(htot)])
        htotx.extend([htot[0][0]])
        htoty.extend([htot[1][0]])
        htotz.extend([htot[2][0]])
        hx.extend([h[0][0]])
        hy.extend([h[1][0]])
        hz.extend([h[2][0]])
        omegasx.extend([i[0]])
        omegasy.extend([i[1]])
        omegasz.extend([i[2]])
        omegadx.extend([i[3]])
        omegady.extend([i[4]])
        omegadz.extend([i[5]])
    return [hx, hy, hz, omegasx, omegasy, omegasz,
            omegadx, omegady, omegadz, htotx, htoty, htotz, hnorm,
            h_norm_spacecraft]

# UKF Measurement model

def quatsFromAngularVelIntegrator(X, t, wx, wy, wz):
    """
    Obtain quaternions from angular velocity
    """
    q1, q2, q3, q4 = X[0], X[1], X[2], X[3]
    q = np.array([[q1, q2, q3, q4]]).T    
    ox, oy, oz = wx(t), wy(t), wz(t)
    omegacrs = np.array([[0., oz, -oy, ox],
                            [-oz, 0., ox, oy],
                            [oy, -ox, 0., oz],
                            [-ox, -oy, -oz, 0.]])
    qdot = .5*np.dot(omegacrs, q)
    
    return [qdot[0][0], qdot[1][0], qdot[2][0], qdot[3][0]]

def obtainAllQuatsfromAngVel(q0, tstop, delta_t, ws):
    """
    Convert omega velocity from 0 to [tstop] into quaternions through integration
    """
    a_t = np.arange(0, tstop, delta_t)
    asol = integrate.odeint(quatsFromAngularVelIntegrator, q0, a_t, args=ws)
    q1, q2, q3, q4 = [], [], [], []
    for i in asol:
        q1.extend([i[0]])
        q2.extend([i[1]])
        q3.extend([i[2]])
        q4.extend([i[3]])
    return [q1, q2, q3, q4]

def plotSpacecraft(omega_init, tstop, delta_t, kickTime):
    """
    Obtain angular velocities from time 0 to [tstop] with initial
    velocity [omega_init] and cold-gas thruster fire time [kickTime].
    """
    time = np.arange(0, tstop, delta_t)
    X = omega_init
    omega = goSpacecraft(X, tstop, delta_t, kickTime)
    omega[0] = np.array(omega[0])
    omega[1] = np.array(omega[1])
    omega[2] = np.array(omega[2])
    return omega;
    
def goBias(tstop, gyro_t, bias_init, gyroSigma, gyroNoiseSigma):
    """
    Generate biases using random noise.
    """
    time = np.arange(0, tstop, gyro_t)
    biasx, biasy, biasz = [bias_init[0]], [bias_init[1]], [bias_init[2]]
    for i in range(len(time)-1):
        xrand = sum(np.random.normal(0, gyroSigma, 100))
        yrand = sum(np.random.normal(0, gyroSigma, 100))
        zrand = sum(np.random.normal(0, gyroSigma, 100))
        biasx.extend([biasx[-1] + xrand + np.random.normal(0, gyroNoiseSigma)])
        biasy.extend([biasy[-1] + yrand + np.random.normal(0, gyroNoiseSigma)])
        biasz.extend([biasz[-1] + zrand + np.random.normal(0, gyroNoiseSigma)])
    bias = [biasx, biasy, biasz]
    return bias

def generateSyntheticData(quat, cameradt, kickTime, omega_init, bias_init, gyro_sample_rate, totalIntegrationTime, gyroSigma, gyroNoiseSigma):
    """
    Generates fake attitude data for sampling.
    [quat]: starting quaternion; should just be a guess
    [cameradt]: time separation between each camera measurement
    [kickTime]: kick time for cold-gas thruster
    [bias_init]: starting gyro bias; should be a guess
    [gyro_sample_rate]: time separation between each gyro measurement
    [totalIntegrationTime]: how long the sequence lasts
    [gyroSigma]: gyro error standard deviation
    [gyroNoiseSigma]: gyro noise error standard deviation
    """
    angular_momentum_history = plotSpacecraft(omega_init, totalIntegrationTime, INTEGRATION_TIMESTEP, kickTime);
    totaltime = np.arange(0, totalIntegrationTime, INTEGRATION_TIMESTEP)
    hx = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[9])
    hy = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[10])
    hz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[11])
    omegax = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[3])
    omegay = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[4])
    omegaz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[5])

    q0 = quat/np.linalg.norm(quat)
    quaternion_history = obtainAllQuatsfromAngVel(q0.flatten(), totalIntegrationTime, INTEGRATION_TIMESTEP, (omegax, omegay, omegaz));
    q1 = InterpolatedUnivariateSpline(totaltime, quaternion_history[0])
    q2 = InterpolatedUnivariateSpline(totaltime, quaternion_history[1])
    q3 = InterpolatedUnivariateSpline(totaltime, quaternion_history[2])
    q4 = InterpolatedUnivariateSpline(totaltime, quaternion_history[3])
    # Propagate bias
    bias_history = goBias(totalIntegrationTime, gyro_sample_rate, bias_init, gyroSigma, gyroNoiseSigma);
    gyrotime = np.arange(0, totalIntegrationTime, gyro_sample_rate)
    biasx = InterpolatedUnivariateSpline(gyrotime, bias_history[0])
    biasy = InterpolatedUnivariateSpline(gyrotime, bias_history[1])
    biasz = InterpolatedUnivariateSpline(gyrotime, bias_history[2])
    
    return q1, q2, q3, q4, omegax, omegay, omegaz, biasx, biasy, biasz

def plotResults(results, q1, q2, q3, q4, biasx, biasy, biasz):
    """
    Plot results from attitude.UKF()
    [results]: Output from attitude.UKF()
    [q1, q2, q3, q4, biasx, biasy, biasz]: Ground truth
    """
    trace = []
    quat1, quat2, quat3, quat4 = [], [], [], []
    true1, true2, true3, true4 = [], [], [], []
    x1, x2, x3 = [], [], []
    b1, b2, b3 = [], [], []
    p1, p2, p3, p4, p5, p6 = [], [], [], [], [], []
    tbias1, tbias2, tbias3 = [], [], []
    truerod1, truerod2, truerod3 = [], [], []
    attitude_error = []
    for i in range(len(results[1])):
        truequaternion = np.array([[q1(i), q2(i), q3(i), q4(i)]]).T
        errquat = quaternionComposition(results[1][i], quaternionInv(truequaternion))
        errquat = errquat/np.linalg.norm(errquat)
        deltaqhat = np.array([[errquat[0][0], errquat[1][0], errquat[2][0]]]).T
        deltaq4 = errquat[3][0]
        attitude_error.extend([2.*np.arccos(deltaq4) * (180./np.pi)])
        rod = 1*(_f*deltaqhat)/(_a + deltaq4)
        truerod1.extend([rod[0][0]])
        truerod2.extend([rod[1][0]])
        truerod3.extend([rod[2][0]])
        p1.extend([results[0][i][0][0]])
        p2.extend([results[0][i][1][1]])
        p3.extend([results[0][i][2][2]])
        p4.extend([results[0][i][3][3]])
        p5.extend([results[0][i][4][4]])
        p6.extend([results[0][i][5][5]])
        tbias1.extend([biasx(i)])
        tbias2.extend([biasy(i)])
        tbias3.extend([biasz(i)])
    for i in results[0]:
        trace.extend([np.trace(i)])
    for i in results[1]:
        quat1.extend([i[0][0]])
        quat2.extend([i[1][0]])
        quat3.extend([i[2][0]])
        quat4.extend([i[3][0]])
    for i in np.arange(0,len(results[1]),1):
        true1.extend([q1(i)])
        true2.extend([q2(i)])
        true3.extend([q3(i)])
        true4.extend([q4(i)])
    for i in results[2]:
        x1.extend([i[0][0]])
        x2.extend([i[1][0]])
        x3.extend([i[2][0]])
        b1.extend([i[3][0]])
        b2.extend([i[4][0]])
        b3.extend([i[5][0]])

    plt.plot(quat1[90:200], 'r-', label='$q_{1}$')
    plt.plot(true1[90:200], 'r:', label='$q^{True}_{1}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(quat2[90:200], 'g-', label='$q_{2}$')
    plt.plot(true2[90:200], 'g:', label='$q^{True}_{2}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(quat3[90:200], 'b-', label='$q_{3}$')
    plt.plot(true3[90:200], 'b:', label='$q^{True}_{3}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
        
    plt.plot(quat4[90:200], 'y-', label='$q_{4}$')
    plt.plot(true4[90:200], 'y:', label='$q^{True}_{4}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(b1, 'r-', label='$b_{1}$')
    plt.plot(b2, 'g-', label='$b_{2}$')
    plt.plot(b3, 'b-', label='$b_{3}$')
    plt.plot(tbias1,'r:', label='$b_{1}^{True}$')
    plt.plot(tbias2,'g:', label='$b_{2}^{True}$')
    plt.plot(tbias3,'b:', label='$b_{3}^{True}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Gyro Bias')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(x1, 'r-', label='$rod_{1}$')
    plt.plot(x2, 'g-', label='$rod_{2}$')
    plt.plot(x3, 'b-', label='$rod_{3}$')
    plt.plot(truerod1,'r:', label='$rod_{1}^{True}$')
    plt.plot(truerod2,'g:', label='$rod_{2}^{True}$')
    plt.plot(truerod3,'b:', label='$rod_{3}^{True}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Rodrigues Parameters')
    plt.xlabel('Seconds')
    plt.ylim([-0.1,0.1])
    plt.show()
    
    plt.plot(trace, label='Trace of $P_k$')
    plt.title('Trace of Covariance Matrix')
    plt.xlabel('$k$')
    plt.yscale('log')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    
    plt.plot(np.array(b1) - np.array(tbias1), 'r-', label='$b_{1}^{error}$')
    plt.plot(np.array(b2) - np.array(tbias2), 'g-', label='$b_{2}^{error}$')
    plt.plot(np.array(b3) - np.array(tbias3), 'b-', label='$b_{3}^{error}$')

    plt.title('Bias Error')
    plt.xlabel('Seconds')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    
    plt.plot(np.array(x1) - np.array(truerod1), 'r-', label='$err_{1}$')
    plt.plot(np.array(x2) - np.array(truerod2), 'g-', label='$err_{2}$')
    plt.plot(np.array(x3) - np.array(truerod3), 'b-', label='$err_{3}$')

    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('True Rodrigues Errors')
    plt.xlabel('Seconds')
    plt.ylim([-0.1,0.1])
    plt.show()
    
    plt.plot(attitude_error, label='Error (deg.)')
    plt.title('Attitude Error for UKF - Degrees')
    plt.xlabel('Seconds')
    plt.ylabel('Degrees')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.yscale('log')
    plt.show()



"""
z1 = em
z2 = es
z3 = ms
z4 = e
z5 = m
z6 = s
"""

def get6HoursBatch(startTime, endTime, coldGasThrustKickTime, cameradt, omegaInit, biasInit, quat, gyroSampleCount, timeNoiseSigma, gyroSigma, gyroNoiseSigma):
    """
        ____________
        Description:
        Creates a new batch of measurements for the 6Hours dataset.
        This dataset is segmented by 6 hours of measurements (@ 1 minute intervals) 
        followed by 6 hours of silence. To obtain the desired cameradt measurement rate 
        and the gyroSampleCount, interpolation is used to obtain the desired quantaties
        at floating-point time values. This allows for a realistic data-gathering
        scheme as ANY sampling rate, including fractional rates, can be simulated.
        ___________
        Parameters:
        [startTime]: start time for the ephemeris tables (s)
        [endTime]: end time for the ephemeris tables, or None for full trajectory (s)
        [mainThrustKickTime]: when main thruster was fired (s)
                            startTime <= mainThrustKickTime <= endTime
        [coldGasThrustKickTime]: when cold-gas thruster was fired (s)
                            startTime <= coldGasThrustKickTime <= endTime
        [cameradt]: time delay between each camera measurement
        [omegaInit]: starting angular velocity of spacecraft with bias (6,)
        [biasInit]: starting bias of gyro (3,)
        [quat]: random initial quaternion (4x1)
        [gyroSampleCount]: number of gyro measurements between each camera measurement
        [timeNoiseSigma]: Each measurement may not be taken exactly at time t. Assume
                          scheduling delay is modeled as Gaussian Normal with mean 0 
                          and std = timeNoiseSigma
        [gyroSigma]: std of gyro measurements
        [gyroNoiseSigma]: std of gyro measurement noise
        ________
        Returns:
        camera measurements, omegas, biases, sunEph, moonEph, trueTraj, mainThrustKickTimes/Amounts/Durations
        ______________
        Table Formats:
        -Camera Measurements-
        let a = # of measurements
        |time |batch|z1|z2|z3|z4|z5|z6|
        |sTime|----0|# |# |# |# |# |# |
        |             ...             |
        |eTime|--a-1|# |# |# |# |# |# |

        -Omegas & Biases-
        let dt = cameradt / gyroSampleCount
        |time |batch|omegax|omegay|omegaz|biasx|biasy|biasz|
        |sTime|----0|   #  |   #  |   #  |  #  |  #  |  #  |
        |sT+dt|----0|   #  |   #  |   #  |  #  |  #  |  #  |
        |                        ...                       |
        |eT-dt|--a-1|   #  |   #  |   #  |  #  |  #  |  #  |
        |eTime|--a-1|   #  |   #  |   #  |  #  |  #  |  #  |

        -Sun/Moon Ephemerides-
        |time |batch| x| y| z|vx|vy|vz|
        |sTime|----0|# |# |# |# |# |# |
        |             ...             |
        |eTime|--a-1|# |# |# |# |# |# |

        -True Trajectory-
        |time |batch| x| y| z|vx|vy|vz|
        |sTime|----0|# |# |# |# |# |# |
        |             ...             |
        |eTime|--a-1|# |# |# |# |# |# |
    """
    trajTruthdf = pd.read_csv(TEST_6HOURS_traj)
    moonEphdf = pd.read_csv(TEST_6HOURS_moonEph)
    sunEphdf = pd.read_csv(TEST_6HOURS_sunEph)
    measEphdf = pd.read_csv(TEST_6HOURS_meas)

    time = []
    currentT = 0
    # We want to capture the time (in secs) for each measurement to construct
    # the interpolated univariate spline object correctly.
    episode = 6 * 60 * 60       # seconds (6 hours * 60 min/hr * 60 sec/min)
    print('----------------------\n6 Hours Dataset\n----------------------\n')
    for t in tqdm(range(moonEphdf['x'].size), desc='Computing timeline'):
        if t % episode == 0 and t is not 0:
            currentT += episode
        time.append(currentT)
        currentT += 60                 # measurements were taken 60 seconds apart

    if endTime == None:
        endTime = time[-1]

    totalIntegrationTime = endTime - startTime

    moonX = InterpolatedUnivariateSpline(time, list(moonEphdf['x']))
    moonY = InterpolatedUnivariateSpline(time, list(moonEphdf['y']))
    moonZ = InterpolatedUnivariateSpline(time, list(moonEphdf['z']))
    moonVX = InterpolatedUnivariateSpline(time, list(moonEphdf['vx']))
    moonVY = InterpolatedUnivariateSpline(time, list(moonEphdf['vy']))
    moonVZ = InterpolatedUnivariateSpline(time, list(moonEphdf['vz']))

    sunX = InterpolatedUnivariateSpline(time, list(sunEphdf['x']))
    sunY = InterpolatedUnivariateSpline(time, list(sunEphdf['y']))
    sunZ = InterpolatedUnivariateSpline(time, list(sunEphdf['z']))
    sunVX = InterpolatedUnivariateSpline(time, list(sunEphdf['vx']))
    sunVY = InterpolatedUnivariateSpline(time, list(sunEphdf['vy']))
    sunVZ = InterpolatedUnivariateSpline(time, list(sunEphdf['vz']))

    trajX = InterpolatedUnivariateSpline(time, list(trajTruthdf['x']))
    trajY = InterpolatedUnivariateSpline(time, list(trajTruthdf['y']))
    trajZ = InterpolatedUnivariateSpline(time, list(trajTruthdf['z']))
    trajVX = InterpolatedUnivariateSpline(time, list(trajTruthdf['vx']))
    trajVY = InterpolatedUnivariateSpline(time, list(trajTruthdf['vy']))
    trajVZ = InterpolatedUnivariateSpline(time, list(trajTruthdf['vz']))

    z1 = InterpolatedUnivariateSpline(time, list(measEphdf['z1']))
    z2 = InterpolatedUnivariateSpline(time, list(measEphdf['z2']))
    z3 = InterpolatedUnivariateSpline(time, list(measEphdf['z3']))
    z4 = InterpolatedUnivariateSpline(time, list(measEphdf['z4']))
    z5 = InterpolatedUnivariateSpline(time, list(measEphdf['z5']))
    z6 = InterpolatedUnivariateSpline(time, list(measEphdf['z6']))

    # Create Dataframes to hold the sampled quantities
    d_camMeas = {'time': [], 'batch': [], 'z1': [], 'z2': [], 'z3': [], 'z4': [], 'z5': [], 'z6': []}
    d_moonEph = {'time': [], 'batch': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': []}
    d_sunEph = {'time': [], 'batch': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': []}
    d_traj = {'time': [], 'batch': [], 'x': [], 'y': [], 'z': [], 'vx': [], 'vy': [], 'vz': []}
    earthVectors = []
    moonVectors = []
    sunVectors = []
    # Sample each quantity
    sampleTimeLine = np.arange(start=startTime, stop=endTime, step=cameradt)
    for i in tqdm(range(len(sampleTimeLine)), desc='Sampling'):
        t = sampleTimeLine[i] + np.random.normal(0,timeNoiseSigma,1)[0]
        d_camMeas['time'].append(t)
        d_camMeas['batch'].append(i)
        d_camMeas['z1'].append(float(z1(t)))
        d_camMeas['z2'].append(float(z2(t)))
        d_camMeas['z3'].append(float(z3(t)))
        d_camMeas['z4'].append(float(z4(t)))
        d_camMeas['z5'].append(float(z5(t)))
        d_camMeas['z6'].append(float(z6(t)))

        t = sampleTimeLine[i]
        d_moonEph['time'].append(t)
        d_moonEph['batch'].append(i)
        d_moonEph['x'].append(float(moonX(t)))
        d_moonEph['y'].append(float(moonY(t)))
        d_moonEph['z'].append(float(moonZ(t)))
        d_moonEph['vx'].append(float(moonVX(t)))
        d_moonEph['vy'].append(float(moonVY(t)))
        d_moonEph['vz'].append(float(moonVZ(t)))

        d_sunEph['time'].append(t)
        d_sunEph['batch'].append(i)
        d_sunEph['x'].append(float(sunX(t)))
        d_sunEph['y'].append(float(sunY(t)))
        d_sunEph['z'].append(float(sunZ(t)))
        d_sunEph['vx'].append(float(sunVX(t)))
        d_sunEph['vy'].append(float(sunVY(t)))
        d_sunEph['vz'].append(float(sunVZ(t)))

        d_traj['time'].append(t)
        d_traj['batch'].append(i)
        d_traj['x'].append(float(trajX(t)))
        d_traj['y'].append(float(trajY(t)))
        d_traj['z'].append(float(trajZ(t)))
        d_traj['vx'].append(float(trajVX(t)))
        d_traj['vy'].append(float(trajVY(t)))
        d_traj['vz'].append(float(trajVZ(t)))

        satPos = np.array([d_traj['x'][t], d_traj['y'][t], d_traj['z'][t]])
        moonPos = np.array([d_moonEph['x'][t], d_moonEph['y'][t], d_moonEph['z'][t]])
        sunPos = np.array([d_sunEph['x'][t], d_sunEph['y'][t], d_sunEph['z'][t]])
        earthVec = (-1*satPos)/np.linalg.norm(satPos)
        moonVec = (moonPos - satPos)/np.linalg.norm(moonPos - satPos)
        sunVec = (sunPos - satPos)/np.linalg.norm(sunPos - satPos)
        earthVectors.append(earthVec)
        moonVectors.append(moonVec)
        sunVectors.append(sunVectors)

    q1, q2, q3, q4, omegax, omegay, omegaz, biasx, biasy, biasz = generateSyntheticData(quat, cameradt, coldGasThrustKickTime, omegaInit, biasInit, 1.0/gyroSampleCount, totalIntegrationTime, gyroSigma, gyroNoiseSigma)
    return q1, q2, q3, q4, omegax, omegay, omegaz, biasx, biasy, biasz, d_camMeas, d_moonEph, d_sunEph, d_traj, earthVectors, moonVectors, sunVectors
    
    # # graph
    # liveTraj = LiveTrajectoryPlot()
    # for t in tqdm(range(len(d_moonEph['time'])), desc='Graphing'):
    #     liveTraj.preRender(text='Ephemeris Spline Data {}'.format(cameradt))
    #     liveTraj.render(d_moonEph['x'][:t], d_moonEph['y'][:t], d_moonEph['z'][:t], color='red', label='Moon Position')
    #     liveTraj.render(d_traj['x'][:t], d_traj['y'][:t], d_traj['z'][:t], color='blue', label='Satellite Position')
    #     # liveTraj.render(d_camMeas['z4'][:t], d_camMeas['z5'][:t], d_camMeas['z6'][:t], color='red', label='Moon Position')
    #     liveTraj.postRender(delay=0.001)
    