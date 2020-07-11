import os
import pandas as pd
import numpy as np

"""
Run python createCZML.py > temp.text
Remove the last comma from temp.text (for the last row, z coordinate)
copy everything and paste into CislunarExplorersCesium/Cesium-1.66/Apps/cislunar.czml in the 'cartesian' section.
Set start and end dates acccordingly.
"""

if __name__ == "__main__":
    INITIAL_TRAJ_PATH = os.path.join('D:', 'OpNav', 'data', 'CislunarFullTraj_60secs', 'trajectory', 'traj.csv')
    trajDf = pd.read_csv(INITIAL_TRAJ_PATH)
    timestep = 60 # traj.csv is in minutes
    time = list(np.arange(0,len(trajDf.index)*60, timestep))
    x = np.array(trajDf['x'].tolist()) * 1000
    y = np.array(trajDf['y'].tolist()) * 1000
    z = np.array(trajDf['z'].tolist()) * 1000

    startDate = "2018-10-09T18:00:00.0000Z"
    endDate = "2019-08-12T17:00:00.0000Z"

    for i in range(len(time)):
        print(f'{time[i]},{x[i]},{y[i]},{z[i]},')
    # endDate = ""

    # for index, row in moonDf.iterrows():
    #     time.append(index)
    #     x.append(trajDf['x'][index])