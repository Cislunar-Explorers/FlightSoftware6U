"""
The purpose of this script is to extend ephemerides that were sampled
at every hour to desired interval.

The 1-hour samples were generating using JPL's HORIZONS program:
https://ssd.jpl.nasa.gov/horizons.cgi#top

Spacecraft coordinate frame is: https://help.agi.com/stk/11.0.1/Content/attitude/attTypes_nadirEci.htm#:~:text=Nadir%20Alignment%20with%20ECI%20Velocity%20Constraint%20Attitude%20Profile,offset%20about%20the%20nadir%20vector.
+X = FORWARD (body frame)
+Z = NADIR (body frame)

Legend: 
Dashed-black arrow: velocity vector (Forward)
Dashed-Red arrow: Rotated X basis vector (using quaternion mult)
Dashed-Green arrow: Rotated Y basis vector (using quaternion mult)
Dashed-Blue arrow: Rotated Z basis vector (using quaternion mult)
Solid-Red arrow: Rotated X basis vector (using attitude matrix)
Solid-Green arrow: Rotated Y basis vector (using attitude matrix)
Solid-Blue arrow: Rotated Z basis vector (using attitude matrix)

Eashaan Kumar, ek485
Summer 2020
"""

import pandas as pd
import numpy as np
import math
import os
from tqdm import tqdm
from scipy.interpolate import InterpolatedUnivariateSpline
from animations import LiveMultipleTrajectoryPlot

def quaternion_multiply(quaternion1, quaternion0):
    """
    Hamiltonian Product
    Source: https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy
    https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
    ai + bj + ck + d
    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    assert abs(np.linalg.norm(quaternion0) - 1) < 1e-6
    assert abs(np.linalg.norm(quaternion1) - 1) < 1e-6
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def attitudeMatrix(quaternion):
    q1, q2, q3, q4 = quaternion
    return np.array([[q1**2-q2**2-q3**2+q4**2, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)], 
                     [2*(q2*q1+q3*q4), -q1**2+q2**2-q3**2+q4**2, 2*(q2*q3-q1*q4)], 
                     [2*(q3*q1-q2*q4), 2*(q3*q2-q1*q4), -q1**2-q2**2+q3**2+q4**2]], dtype=np.float64)

def changeEph(path, sample_rate):
    """
    Samples ephemeris or trajectory from original table [path] at new [sample_rate].
    For example, if original sun ephemeris table was sampled at 1 hour intervals, this
    function will return an ephemeris table sampled at 1 min intervals if sample_rate=1./60.0.
    
    Precondition: file at [path] must be a csv that contains 6 columns (x, y, z, vx, vy, vz).

    Returns: original file size * 1/sample_rate by len(columns) numpy array
    """

    assert sample_rate > 0

    Ephdf = pd.read_csv(path)
    m = np.zeros((len(Ephdf.index),6))

    for index, row in tqdm(Ephdf.iterrows(), total=Ephdf.shape[0]):
        m[index, 0] = float(row['x'])
        m[index, 1] = float(row['y'])
        m[index, 2] = float(row['z'])
        m[index, 3] = float(row['vx'])
        m[index, 4] = float(row['vy'])
        m[index, 5] = float(row['vz'])

    totaltime = np.arange(0, m.shape[0], 1)
    mx = InterpolatedUnivariateSpline(totaltime, m[:,0])
    my = InterpolatedUnivariateSpline(totaltime, m[:,1])
    mz = InterpolatedUnivariateSpline(totaltime, m[:,2])
    mvx = InterpolatedUnivariateSpline(totaltime, m[:,3])
    mvy = InterpolatedUnivariateSpline(totaltime, m[:,4])
    mvz = InterpolatedUnivariateSpline(totaltime, m[:,5])
    
    sample_timeline = np.arange(0, m.shape[0], sample_rate)
    x = np.zeros((sample_timeline.shape[0]))
    y = np.zeros((sample_timeline.shape[0]))
    z = np.zeros((sample_timeline.shape[0]))
    vx = np.zeros((sample_timeline.shape[0]))
    vy = np.zeros((sample_timeline.shape[0]))
    vz = np.zeros((sample_timeline.shape[0]))

    for i,t in enumerate(tqdm(sample_timeline)):
        x[i] = mx(t)
        y[i] = my(t)
        z[i] = mz(t)
        vx[i] = mvx(t)
        vy[i] = mvy(t)
        vz[i] = mvz(t)

    d = {'x': x, 'y':y, 'z':z, 'vx':vx, 'vy':vy,'vz':vz}
    return pd.DataFrame(d, columns = ['x','y','z','vx','vy','vz'])

def getRotatedVector(local_vector, q):
    q_1 = [-q[0], -q[1], -q[2], q[3]]
    q_norm = math.sqrt(q_1[0]**2 + q_1[1]**2 + q_1[2]**2 + q_1[3]**2)
    q_1 = [q_1[0]/q_norm, q_1[1]/q_norm, q_1[2]/q_norm, q_1[3]/q_norm]
    a = quaternion_multiply(q, local_vector) # drop the w component
    b = quaternion_multiply(a,q_1)
    assert abs(np.linalg.norm(a) - 1) < 1e-3
    assert abs(np.linalg.norm(b) - 1) < 1e-3
    return b[:3]

def isOrthogonal(a, b):
    err = abs((a*b).sum())
    return [err < 1e-3, err]

if __name__ == "__main__":
    INITIAL_MOON_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', '1hr_moon_eph.csv')
    INITIAL_SUN_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', '1hr_sun_eph.csv')
    INITIAL_TRAJ_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'trajectory', 'traj.csv')
    df = changeEph(INITIAL_TRAJ_PATH, 1./60.)
    df.to_csv(os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'trajectory', 'trajsec.csv'))

    # # INITIAL_ATT_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'attitude', 'attitude.csv')
    # # SAMPLED_MOON_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', 'sampled_moon_eph.csv')
    # # SAMPLED_SUN_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', 'sampled_sun_eph.csv')
    
    # # moonDf = changeEph(INITIAL_MOON_PATH, 1./60.)
    # # sunDf = changeEph(INITIAL_SUN_PATH, 1./60.)
    # moonDf = pd.read_csv(SAMPLED_MOON_PATH)
    # sunDf = pd.read_csv(SAMPLED_SUN_PATH)
    # trajDf = pd.read_csv(INITIAL_TRAJ_PATH)
    # attDf = pd.read_csv(INITIAL_ATT_PATH)
    # assert len(moonDf.index) == len(sunDf.index) == len(trajDf.index) == len(attDf.index)
    # # moonDf.to_csv(SAMPLED_MOON_PATH, index=False)
    # # sunDf.to_csv(SAMPLED_SUN_PATH, index=False)

    # liveTraj = LiveMultipleTrajectoryPlot(trajectories=2, trackingArrows=7)
    # liveTraj.setTrajectorySettings(0, 'red', 'moon 1 min', 0.5)
    # # liveTraj.setTrajectorySettings(1, 'blue', 'sun 1 min', 0.5)
    # liveTraj.setTrajectorySettings(1, 'green', 'sat 1 min', 0.5)
    # liveTraj.setTrackingArrowSettings(0, style="-|>", color="r", lw=1, ls='dashed')
    # liveTraj.setTrackingArrowSettings(1, style="-|>", color="g", lw=1, ls='dashed')
    # liveTraj.setTrackingArrowSettings(2, style="-|>", color="b", lw=1, ls='dashed')
    # liveTraj.setTrackingArrowSettings(3, style="-|>", color="r", lw=1, ls='-')
    # liveTraj.setTrackingArrowSettings(4, style="-|>", color="g", lw=1, ls='-')
    # liveTraj.setTrackingArrowSettings(5, style="-|>", color="b", lw=1, ls='-')
    # liveTraj.setTrackingArrowSettings(6, style="-|>", color="black", lw=1, ls='dashed')

    # for index, row in moonDf.iterrows():
    #     # print(moonDf['x'][index], sunDf['x'][index])
    #     if index % (60) == 0: # sample at hour interval
    #         liveTraj.updateTraj(0, moonDf['x'][index], moonDf['y'][index], moonDf['z'][index])
    #         # liveTraj.updateTraj(1, sunDf['x'][index], sunDf['y'][index], sunDf['z'][index])
    #         liveTraj.updateTraj(1, trajDf['x'][index], trajDf['y'][index], trajDf['z'][index])
    #         start_pos = np.array([trajDf['x'][index], trajDf['y'][index], trajDf['z'][index]])
    #         # Velocity
    #         new_vector = np.array([trajDf['vx'][index], trajDf['vy'][index], trajDf['vz'][index]])
    #         new_vector /= np.linalg.norm(new_vector)
    #         liveTraj.updateAtt(6, start_pos, new_vector)

    #         # Extract quaternions
    #         q = np.array([attDf['q1'][index], attDf['q2'][index],attDf['q3'][index],attDf['q4'][index]])
    #         assert abs(np.linalg.norm(q) - 1) < 1e-3
    #         Aq = attitudeMatrix(q)

    #         # X axis of spacecraft
    #         local_vector = np.array([1, 0, 0, 0]) # last 0 is padding
    #         X_qm = getRotatedVector(local_vector, q.copy())
    #         X_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

    #         # assert abs(np.linalg.norm(X_qm) - 1) < 1e-6
    #         # assert abs(np.linalg.norm(X_A) - 1) < 1e-3

    #         # Y axis of spacecraft
    #         local_vector = np.array([0, 1, 0, 0]) # last 0 is padding
    #         Y_qm = getRotatedVector(local_vector, q.copy())
    #         Y_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

    #         # assert abs(np.linalg.norm(Y_qm) - 1) < 1e-6
    #         # assert abs(np.linalg.norm(Y_A) - 1) < 1e-3

    #         # Z axis of spacecraft
    #         local_vector = np.array([0, 0, 1, 0]) # last 0 is padding
    #         Z_qm = getRotatedVector(local_vector, q.copy())
    #         Z_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

    #         # assert abs(np.linalg.norm(Z_qm) - 1) < 1e-6
    #         # assert abs(np.linalg.norm(Z_A) - 1) < 1e-3

    #         # XYZ should be orthonormal
    #         print(isOrthogonal(X_A, Y_A)[1], isOrthogonal(X_A, Z_A)[1], isOrthogonal(Y_A, Z_A)[1])
    #         print(isOrthogonal(X_qm, Y_qm)[1], isOrthogonal(X_qm, Z_qm)[1], isOrthogonal(Y_qm, Z_qm)[1])
    #         # assert isOrthogonal(X_A, Y_A)[0] and isOrthogonal(X_A, Z_A)[0] and isOrthogonal(Y_A, Z_A)[0] 
    #         # assert isOrthogonal(X_qm, Y_qm)[0] and isOrthogonal(X_qm, Z_qm)[0] and isOrthogonal(Y_qm, Z_qm)[0]

    #         # liveTraj.updateAtt(0, start_pos, X_qm)
    #         # liveTraj.updateAtt(1, start_pos, Y_qm)
    #         # liveTraj.updateAtt(2, start_pos, Z_qm)
    #         liveTraj.updateAtt(3, start_pos, X_A)
    #         liveTraj.updateAtt(4, start_pos, Y_A)
    #         liveTraj.updateAtt(5, start_pos, Z_A)


    #         liveTraj.renderUKF(text="{}/{}".format(index,len(moonDf.index)),delay=0.01)

