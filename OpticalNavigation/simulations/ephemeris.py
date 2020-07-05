"""
The purpose of this script is to extend ephemerides that were sampled
at every hour to desired interval.

The 1-hour samples were generating using JPL's HORIZONS program:
https://ssd.jpl.nasa.gov/horizons.cgi#top

Eashaan Kumar, ek485
Summer 2020
"""

import pandas as pd
import numpy as np
import os
from tqdm import tqdm
from scipy.interpolate import InterpolatedUnivariateSpline
from animations import LiveMultipleTrajectoryPlot

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

    for index, row in Ephdf.iterrows():
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

if __name__ == "__main__":
    # INITIAL_MOON_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', '1hr_moon_eph.csv')
    # INITIAL_SUN_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', '1hr_sun_eph.csv')
    INITIAL_TRAJ_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'trajectory', 'traj.csv')

    SAMPLED_MOON_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', 'sampled_moon_eph.csv')
    SAMPLED_SUN_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'ephemeris', 'sampled_sun_eph.csv')
    
    # moonDf = changeEph(INITIAL_MOON_PATH, 1./60.)
    # sunDf = changeEph(INITIAL_SUN_PATH, 1./60.)
    moonDf = pd.read_csv(SAMPLED_MOON_PATH)
    sunDf = pd.read_csv(SAMPLED_SUN_PATH)
    trajDf = pd.read_csv(INITIAL_TRAJ_PATH)
    print(len(moonDf.index), len(sunDf.index), len(trajDf.index))
    assert len(moonDf.index) == len(sunDf.index) == len(trajDf.index)
    # moonDf.to_csv(SAMPLED_MOON_PATH, index=False)
    # sunDf.to_csv(SAMPLED_SUN_PATH, index=False)

    liveTraj = LiveMultipleTrajectoryPlot(2)
    liveTraj.setTrajectorySettings(0, 'red', 'moon 1 day', 0.5)
    liveTraj.setTrajectorySettings(1, 'blue', 'sun 1 day', 0.5)
    for index, row in moonDf.iterrows():
        # print(moonDf['x'][index], sunDf['x'][index])
        if index % (60*24) == 0: # sample at day interval
            liveTraj.updateTraj(0, moonDf['x'][index], moonDf['y'][index], moonDf['z'][index])
            liveTraj.updateTraj(1, sunDf['x'][index], sunDf['y'][index], sunDf['z'][index])

            liveTraj.renderUKF(text="{}/{}".format(index,len(moonDf.index)),delay=0.01)

