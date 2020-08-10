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
from animations import LiveMultipleTrajectoryPlot
import argparse
from getManeuvers import extractCheckpoints, getMissionTimeline, createDiscreteAttitudeManeuvers
from datetime import datetime
from core.ukf import attitudeMatrix

def isOrthogonal(a, b):
    err = abs((a*b).sum())
    return [err < 1e-3, err]

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--opnavdataset", help = "path to OpNav Dataset root folder")
    ap.add_argument("-r", "--samplingRate", help = "sampling rate of ephemeris/traj to seconds. Eg: if data is in 1-min interval, -r=60")
    ap.add_argument("-m", "--multiplier", help = "simulation multiplier")
    args = vars(ap.parse_args())

    INITIAL_TRAJ_PATH = os.path.join(args['opnavdataset'], 'trajectory', '1min_stk_active_sampled_traj.csv')
    INITIAL_VNC_PATH = os.path.join(args['opnavdataset'], 'attitude', '1min_sampled_vncearth_stk_active_attitude.csv')
    INITIAL_ATT_PATH = os.path.join(args['opnavdataset'], 'attitude', 'discrete_att_with_maneuver_nutations.csv')
    SAMPLED_MOON_PATH = os.path.join(args['opnavdataset'], 'ephemeris', '1min_stk_active_sampled_moon_eph.csv')
    SAMPLED_SUN_PATH = os.path.join(args['opnavdataset'], 'ephemeris', '1min_stk_active_sampled_moon_eph.csv')
    MANEUVER_CHECKPOINT_PATH = os.path.join(args['opnavdataset'], 'maneuvers','checkpoints.csv')
    START_END_DATES_PATH = os.path.join(args['opnavdataset'], 'startEndDates.csv')

    simSamplingRate = float(args['samplingRate'])
    simMultiplier = float(args['multiplier'])
    
    moonDf = pd.read_csv(SAMPLED_MOON_PATH)
    # sunDf = pd.read_csv(SAMPLED_SUN_PATH)
    trajDf = pd.read_csv(INITIAL_TRAJ_PATH)
    vncDf = pd.read_csv(INITIAL_VNC_PATH)
    manCheckDf = pd.read_csv(MANEUVER_CHECKPOINT_PATH)
    startEndDatesDf = pd.read_csv(START_END_DATES_PATH)

    # Obtain mission timeline (in seconds)
    missionStartDate, missionEndDate = getMissionTimeline(startEndDatesDf)
    maneuversDict = extractCheckpoints(manCheckDf, missionEndDate)
    # print(missionStartDate, missionEndDate, simSamplingRate)
    missionTimeline = np.arange(missionStartDate, missionEndDate+simSamplingRate, simSamplingRate)

    # Obtain mission attitude data
    # Uncomment the following 3 lines to generate attitude data for simulation (eg. if you have a new dataset, if you want a different spin rate)...
    # missionParams = createDiscreteAttitudeManeuvers(maneuversDict, vncDf, missionStartDate, missionEndDate, missionTimeline)
    # attDf = pd.DataFrame.from_dict(missionParams)
    # attDf.to_csv(os.path.join(args['opnavdataset'], 'attitude', 'att.csv'), index=False)
    # .... and comment out this line
    attDf = pd.read_csv(INITIAL_ATT_PATH)

    minX = min([np.min(moonDf['x'].values), np.min(trajDf['x'].values)])
    minY = min([np.min(moonDf['y'].values), np.min(trajDf['y'].values)])
    minZ = min([np.min(moonDf['z'].values), np.min(trajDf['z'].values)])

    maxX = max([np.max(moonDf['x'].values), np.max(trajDf['x'].values)])
    maxY = max([np.max(moonDf['y'].values), np.max(trajDf['y'].values)])
    maxZ = max([np.max(moonDf['z'].values), np.max(trajDf['z'].values)])

    minB = min(minX, minY, minZ)
    maxB = max(maxX, maxY, maxZ)

    liveTraj = LiveMultipleTrajectoryPlot(bounds=[minX, maxX, minY, maxY, minZ, maxZ])
    moonEphIndex = liveTraj.addTrajectory(color='red', label='moon 1 min eph', alpha=0.5, ls=':', traceLim=100, blobsize=5, blobshape='o', bloblabel='The Moon')
    satTrajIndex = liveTraj.addTrajectory(color='green', label='sat 1 min traj', alpha=0.5, ls=':', traceLim=100000000, blobsize=10, blobshape='$L$', bloblabel='CisLunar Explorer')
    earthBlobIndex = liveTraj.addTrajectory(color='blue', label=None, alpha=0.75, ls=None, traceLim=0, blobsize=20, blobshape='o', bloblabel='The Earth')

    xVNCIndex = liveTraj.addTrackingArrow(style="-|>", color="orange", lw=1, ls='-', name='X_{VNC}', size=30)
    yVNCIndex = liveTraj.addTrackingArrow(style="-|>", color="orange", lw=1, ls='-', name='Y_{VNC}', size=30)
    zVNCIndex = liveTraj.addTrackingArrow(style="-|>", color="orange", lw=1, ls='-', name='Z_{VNC}', size=30)
    velAxisIndex = liveTraj.addTrackingArrow(style="-|>", color="black", lw=1, ls='dashed', name='V_B', size=20)

    xBodyIndex = liveTraj.addTrackingArrow(style="-|>", color="lime", lw=1, ls='-', name='X_B', size=20)
    yBodyIndex = liveTraj.addTrackingArrow(style="-|>", color="lime", lw=1, ls='-', name='Y_B', size=20)
    zBodyIndex = liveTraj.addTrackingArrow(style="-|>", color="lime", lw=1, ls='-', name='Z_B', size=20)

    maneuverArrowIndexMap = [] # checkpoint -> arrow index
    for i in range(len(maneuversDict['startTime'])):
        start_index = liveTraj.addTrajectory(color='magenta', label=None, alpha=0.50, ls='-', traceLim=10000000, blobsize=12, blobshape=f'${i+1}$', bloblabel=f'Maneuver {i+1}')
        maneuverArrowIndexMap.append(start_index)

    # Earth doesn't move
    liveTraj.updateTraj(earthBlobIndex, 0, 0, 0)

    currentManeuver = 0
    isManeuver = False

    satq = np.array([0,0,0,1])

    for index, timestamp in enumerate(missionTimeline):
        if index % (simMultiplier) == 0: # sample at hour interval
            currentDateTime = datetime.fromtimestamp(timestamp)
            liveTraj.updateTraj(moonEphIndex, moonDf['x'][index], moonDf['y'][index], moonDf['z'][index])
            # liveTraj.updateTraj(1, sunDf['x'][index], sunDf['y'][index], sunDf['z'][index])
            liveTraj.updateTraj(satTrajIndex, trajDf['x'][index], trajDf['y'][index], trajDf['z'][index])
            start_pos = np.array([trajDf['x'][index], trajDf['y'][index], trajDf['z'][index]])
            # Velocity
            new_vector = np.array([trajDf['vx'][index], trajDf['vy'][index], trajDf['vz'][index]])
            new_vector /= np.linalg.norm(new_vector)
            liveTraj.updateAtt(velAxisIndex, start_pos, new_vector)

            # Maneuver
            if currentManeuver >= 0 and currentManeuver < len(maneuversDict['startTime']):
                isManeuver = timestamp >= maneuversDict['startTime'][currentManeuver] and timestamp <= maneuversDict['endTime'][currentManeuver]
                if isManeuver:
                    liveTraj.updateTraj(maneuverArrowIndexMap[currentManeuver], start_pos[0], start_pos[1], start_pos[2])
                elif timestamp > maneuversDict['endTime'][currentManeuver]:
                    currentManeuver += 1
            else:
                isManeuver = False

            # Extract VNC quaternions
            q = np.array([vncDf['q1'][index], vncDf['q2'][index],vncDf['q3'][index],vncDf['q4'][index]])
            assert abs(np.linalg.norm(q) - 1) < 1e-3
            Aq = attitudeMatrix(q)
            # X axis of spacecraft
            local_vector = np.array([1, 0, 0, 0]) # last 0 is padding
            X_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
            # Y axis of spacecraft
            local_vector = np.array([0, 1, 0, 0]) # last 0 is padding
            Y_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
            # Z axis of spacecraft
            local_vector = np.array([0, 0, 1, 0]) # last 0 is padding
            Z_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
            # XYZ should be orthonormal
            assert isOrthogonal(X_A, Y_A)[0] and isOrthogonal(X_A, Z_A)[0] and isOrthogonal(Y_A, Z_A)[0] 
            liveTraj.updateAtt(xVNCIndex, start_pos, X_A/np.linalg.norm(X_A))
            liveTraj.updateAtt(yVNCIndex, start_pos, Y_A/np.linalg.norm(Y_A))
            liveTraj.updateAtt(zVNCIndex, start_pos, Z_A/np.linalg.norm(Z_A))

            # Extract body quaternions
            q = np.array([attDf['q1'][index], attDf['q2'][index],attDf['q3'][index],attDf['q4'][index]])
            assert abs(np.linalg.norm(q) - 1) < 1e-3
            Aq = attitudeMatrix(q)
            # X axis of spacecraft
            local_vector = np.array([1, 0, 0, 0]) # last 0 is padding
            X_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
            # Y axis of spacecraft
            local_vector = np.array([0, 1, 0, 0]) # last 0 is padding
            Y_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
            # Z axis of spacecraft
            local_vector = np.array([0, 0, 1, 0]) # last 0 is padding
            Z_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))
            # XYZ should be orthonormal
            assert isOrthogonal(X_A, Y_A)[0] and isOrthogonal(X_A, Z_A)[0] and isOrthogonal(Y_A, Z_A)[0] 
            liveTraj.updateAtt(xBodyIndex, start_pos, X_A/np.linalg.norm(X_A))
            liveTraj.updateAtt(yBodyIndex, start_pos, Y_A/np.linalg.norm(Y_A))
            liveTraj.updateAtt(zBodyIndex, start_pos, Z_A/np.linalg.norm(Z_A))

            liveTraj.renderUKF(text="{}%\n{}\nx{}".format(round(index/len(missionTimeline)*100,2),currentDateTime, simMultiplier),delay=0.001)

    liveTraj.close()

