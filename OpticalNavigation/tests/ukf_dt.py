from core.const import (
    CameraMeasurementVector,
    CovarianceMatrix,
    EphemerisVector,
    TrajectoryStateVector,
    Vector6,
)
import numpy as np

from core.ukf import runTrajUKF
from core.opnav import calculate_cam_measurements
import unittest
from parameterized import parameterized
import logging
import json
import pandas as pd
import math

"""
Tests accuracy of our trajectory ukf on a trivial case. Test takes data from various constants and low initial
velocities and positions to check that we obtained reasonable outputs. The final result is then compared with
our expected outputs (using the standard deviations of our covariance matrix).

moonEph data file: https://cornell.app.box.com/file/651831179494
sunEph data file: https://cornell.app.box.com/file/651825552217
trajStateVector data file: https://cornell.app.box.com/file/651842666983

inputs:
dynamicsModelOnly means we don't need the cameraMeasurements and only rely on initial state ?
note: need to verify whether the camera measurements' units are still
in pixel separation or not; they should have been changed to angular separation

moonEph = get_ephemeris(0, BodyEnum.Moon)  # Moon ephemeris data from astro.py
sunEph = get_ephemeris(0, BodyEnum.Sun)   # Sun ephemeris data from astro.py
dt = seconds
P = Initial covariance matrix

has new_state : TrajectoryStateVector, new_P : CovarianceMatrix, K: Matrix6x6 (K = Kalman Gain)
expected outputs for state (trajStateVector)
one way to test outputs are correct is to separately call functions from ukf.py:

"""

    def file_to_list(filename):
        tests_3600_dt_file = []
        P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))

        with open(filename, "r") as data:
            df = pd.read_csv(data)

        # These observed start times will not be used due to differences in truth
        # data and the generated ephemerides.
        # init_time = ["2020-06-27T21:08:03.0212"]
        # t = Time(init_time, format="isot", scale="tdb")

        def divide_1000(original_traj_vec):
            return original_traj_vec / 1000

        for index in range(0, df.shape[0] - 1):
            row = df.iloc[[index]]
            logging.debug(row["x"])
            trajStateVect = TrajectoryStateVector(
                divide_1000(row["x"]),
                divide_1000(row["y"]),
                divide_1000(row["z"]),
                divide_1000(row["vx"]),
                divide_1000(row["vy"]),
                divide_1000(row["vz"]),
            )
            row_skip = df.iloc[[index + 1]]
            logging.debug(row_skip["x"])
            truthStateVector = TrajectoryStateVector(
                divide_1000(row_skip["x"]),
                divide_1000(row_skip["y"]),
                divide_1000(row_skip["z"]),
                divide_1000(row_skip["vx"]),
                divide_1000(row_skip["vy"]),
                divide_1000(row_skip["vz"]),
            )
            # ephemeris vectors will be generated from the positions only,
            # velocities will all be set to 0.
            tests_3600_dt_file.append(
                [
                    Vector6(row["mx"], row["my"], row["mz"], 0, 0, 0),
                    Vector6(row["sx"], row["sy"], row["sz"], 0, 0, 0),
                    trajStateVect,
                    3600,
                    P,
                    None,
                    truthStateVector,
                ]
            )
        return tests_3600_dt_file

    tests_3600_dt = file_to_list("traj2.csv")

    @parameterized.expand(tests_3600_dt)
    def test_iterative_truth(
        self,
        moonEph,
        sunEph,
        trajStateVector,
        dt,
        P,
        cameraMeasurements,
        truthStateVector,
    ):
        """Iterates through every row of trajectory data and compares following
        row with UKF output."""
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
        pos_array = trajEstimateOutput.new_state.get_position_data()
        vel_array = trajEstimateOutput.new_state.get_velocity_data()
        logging.debug(f"x,y,z position = {pos_array}")
        logging.debug(f"x,y,z velocity =  {vel_array}")
        # expected output for covariance matrix
        logging.debug(f"out covariance = \n {trajEstimateOutput.new_P}")
        # kalman gain is all 0s -->
        logging.debug(f"out kalman = \n {trajEstimateOutput.K}")
        logging.debug("\n")

        # validate output
        # deviation = np.matmul(P, np.ones(6).T)  # 6 x 1 col vector
        # upper = np.add(deviation, trajStateVector.data.T)
        # lower = np.subtract(trajStateVector.data.T, deviation)

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

        # for some reason the calculations below do not match with the calculations
        # in lines 208-212
        # posError = math.sqrt( np.sum((trajNew[:3] - truthState[:3])**2) )
        # velError = math.sqrt( np.sum((trajNew[3:6] - truthState[3:6])**2) )

        logging.debug(f"Position error: {posError}\nVelocity error: {velError}")
        # demo prints
        if posError > TestSequence.pos_error_val:
            print(f"Position error: {posError}\nVelocity error: {velError}\n")

        assert posError <= TestSequence.pos_error_val, "Position error is too large"
        assert velError <= TestSequence.vel_error_val, "Velocity error is too large"

        # assert np.all(np.less_equal(lower, trajEstimateOutput.new_state.data.T))
        # assert np.all(np.less_equal(trajEstimateOutput.new_state.data.T, upper))

    def test_recursive(self):
        """Passes in the truth state at t=0 and compares final state at final
        time to truth data in trajectory data. Calls UKF iteratively and manually
        passes in UKF output as the next input for the UKF again."""
        trajStateVector = TrajectoryStateVector(
            -2.7699842997503623e4,
            -1.4293835289027357e4,
            1713.3139597599213,
            -2.8869995362010077,
            -3.7832739597202594,
            -0.9225267990120935,
        )
        dynamicsOnly = True
        main_thrust_info = None
        cameraMeasurements = None
        P = CovarianceMatrix(
            np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))
        )
        with open("traj2.csv", "r") as data:
            df = pd.read_csv(data)

        for index in range(0, df.shape[0] - 1):
            row = df.iloc[[index]]
            trajEstimateOutput = runTrajUKF(
                Vector6(row["mx"], row["my"], row["mz"], 0, 0, 0),
                Vector6(row["sx"], row["sy"], row["sz"], 0, 0, 0),
                cameraMeasurements,
                trajStateVector,
                3600,
                P,
                main_thrust_info,
                dynamicsOnly,
            )
            trajStateVector = trajEstimateOutput.new_state
            P = trajEstimateOutput.new_P

        trajNew = trajStateVector.data
        truthState = (
            TrajectoryStateVector(
                -1.0680802747723334e5,
                -2.8316336297967434e5,
                -1.0300865709131669e5,
                -0.00874577544564751,
                -0.6036563933415423,
                -0.28878799462247133,
            )
        ).data

        posError = np.linalg.norm(trajNew[0:3] - truthState[0:3])
        velError = np.linalg.norm(trajNew[3:6] - truthState[3:6])
        logging.debug(f"Position error: {posError}\nVelocity error: {velError}\n")

        # demo prints
        if posError > TestSequence.pos_error_val:
            logging.debug(f"Position error: {posError}\nVelocity error: {velError}\n")

        assert posError <= TestSequence.pos_error_val, "Position error is too large"
        assert velError <= TestSequence.vel_error_val, "Velocity error is too large"


if __name__ == "__main__":
    unittest.main()
