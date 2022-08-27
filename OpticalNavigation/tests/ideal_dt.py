"""
Returns a csv file with data containing dt values and the respective errors and t values
Based off of traj2.csv
"""

from core.const import (
    CovarianceMatrix,
    TrajectoryStateVector,
    Vector6,
)
import numpy as np

from core.ukf import runTrajUKF
import logging
import pandas as pd
import math
import matplotlib.pyplot as plt


def file_to_lists(filename):
    tests_3600_dt_file = []
    # error covariance matrix values came from past tests
    # (refer to test_ukf.py)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))

    with open(filename, "r") as data:
        df = pd.read_csv(data)

    # These observed start times will not be used due to differences in truth
    # data and the generated ephemerides.
    # init_time = ["2020-06-27T21:08:03.0212"]
    # t = Time(init_time, format="isot", scale="tdb")

    t_list = df["t"].values.tolist()

    for index in range(0, df.shape[0] - 1):
        row = df.iloc[[index]]
        logging.debug(row["x"])
        trajStateVect = TrajectoryStateVector(
            (row["x"]) / 1000,
            (row["y"]) / 1000,
            (row["z"]) / 1000,
            (row["vx"]) / 1000,
            (row["vy"]) / 1000,
            (row["vz"]) / 1000,
        )
        row_skip = df.iloc[[index + 1]]
        logging.debug(row_skip["x"])
        truthStateVector = TrajectoryStateVector(
            (row_skip["x"]) / 1000,
            (row_skip["y"]) / 1000,
            (row_skip["z"]) / 1000,
            (row_skip["vx"]) / 1000,
            (row_skip["vy"]) / 1000,
            (row_skip["vz"]) / 1000,
        )
        # ephemeris vectors will be generated from the positions only,
        # velocities will all be set to 0.
        tests_3600_dt_file.append(
            [
                Vector6(row["mx"], row["my"], row["mz"], 0, 0, 0),
                Vector6(row["sx"], row["sy"], row["sz"], 0, 0, 0),
                trajStateVect,
                3600,  # unneeded since we are testing different dt values
                P,
                None,
                truthStateVector,
            ]
        )
    return (t_list, tests_3600_dt_file)


def error_arrays(dt, trajectory_data):
    pos_error_array = np.empty(59)
    vel_error_array = np.empty(59)
    pos_error_array[0] = 0

    idx = 1
    for traj_info in trajectory_data:
        moonEph = traj_info[0]
        sunEph = traj_info[1]
        trajStateVector = traj_info[2]
        P = traj_info[4]
        truthStateVector = traj_info[6]
        cameraMeasurements = None
        main_thrust_info = None
        dynamicsOnly = True

        trajEstimateOutput = runTrajUKF(
            moonEph,
            sunEph,
            cameraMeasurements,
            trajStateVector,
            dt,
            CovarianceMatrix(P),
            main_thrust_info,
            dynamicsOnly,
        )
        trajNew = trajEstimateOutput.new_state.data
        truthState = truthStateVector.data
        #  pos_array = trajEstimateOutput.new_state.get_position_data()
        #  vel_array = trajEstimateOutput.new_state.get_velocity_data()
        posError = math.sqrt(
            (trajNew[0] - truthState[0]) ** 2
            + (trajNew[1] - truthState[1]) ** 2
            + (trajNew[2] - truthState[2]) ** 2
        )
        velError = math.sqrt(
            (trajNew[3] - truthState[3]) ** 2
            + (trajNew[4] - truthState[4]) ** 2
            + (trajNew[5] - truthState[5]) ** 2
        )
        pos_error_array[idx] = posError
        vel_error_array[idx] = velError
        idx += 1

    return (pos_error_array, vel_error_array)


dt_values = [1, 2, 5, 10, 15, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300]

data_lists = file_to_lists("traj2.csv")
t_array = np.array(data_lists[0])
trajectory_data = data_lists[1]

for dt in dt_values:
    pos_error = (error_arrays(dt, trajectory_data))[0]
    plt.plot(t_array, pos_error, label=str(dt))


plt.xlabel("Time (s)")
plt.ylabel("Position Error (km)")
plt.title("Position Error vs Time")
plt.legend(title="dt values (s)")
plt.show()
