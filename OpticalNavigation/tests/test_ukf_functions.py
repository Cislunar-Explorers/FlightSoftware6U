from core.const import (
    CameraMeasurementVector,
    CovarianceMatrix,
    EphemerisVector,
    TrajectoryStateVector,
    Vector6,
)
import numpy as np

from core.ukf import runTrajUKF
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


def MSE(observedStateVector, expectedStateVector):
    assert observedStateVector.to_array().size == expectedStateVector.to_array().size
    error_squared_velo = 0
    error_squared_pos = 0
    for index in range(0, observedStateVector.get_position_data().size):
        error_squared_pos = (
            error_squared_pos
            + (
                observedStateVector.get_position_data()[index]
                - expectedStateVector.get_position_data()[index]
            )
            ** 2
        )
        error_squared_velo = (
            error_squared_velo
            + (
                observedStateVector.get_velocity_data()[index]
                - expectedStateVector.get_velocity_data()[index]
            )
            ** 2
        )
    MSE_pos = (
        math.sqrt(error_squared_pos) / observedStateVector.get_position_data().size
    )
    MSE_velo = (
        math.sqrt(error_squared_velo) / observedStateVector.get_velocity_data().size
    )
    return MSE_pos, MSE_velo


def angular_separation(v1, v2):
    dot_prod = np.dot(v1, v2)
    mag1 = np.linalg.norm(v1)
    mag2 = np.linalg.norm(v2)
    return np.arccos(dot_prod / (mag1 * mag2))


class TestSequence(unittest.TestCase):

    """
    to add more tests:
    add moonEph, sunEph, trajStateVector
    add [moonEph, sunEph, trajStateVector, dt, P] in @parameterized.expand
    """

    """###################### begin test data ################################"""
    # TEST 1: Test with no camera measurements
    moonEph1 = EphemerisVector(
        1.5363e05, -3.7237e05, 2887.6, 0.90889, 0.34863, -0.088026
    )
    sunEph1 = EphemerisVector(
        -3.0672e07, -1.441e08, 6670.4, 29.633, -6.0859, -0.00088015
    )
    trajStateVector1 = TrajectoryStateVector(
        1.433e05, 5.3541e05, 1.9355e05, -0.93198, -0.077729, 0.049492
    )

    dt1 = 1
    P1 = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))

    # TEST 2: Test with camera measurements
    moonEph2 = EphemerisVector(
        1.5363e05, -3.7237e05, 2887.6, 0.90889, 0.34863, -0.088026
    )
    sunEph2 = EphemerisVector(
        -3.0672e07, -1.441e08, 6670.4, 29.633, -6.0859, -0.00088015
    )
    trajStateVector2 = TrajectoryStateVector(
        1.433e05, 5.3541e05, 1.9355e05, -0.93198, -0.077729, 0.049492
    )

    dt2 = 1
    P2 = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))

    # Data obtained from simulations/sim/data/trajectory_sim/observations.json | time: 10800.0
    earth_vec = np.array(
        [-0.7536623262310652, -0.0018752356256416247, 0.657259143345551]
    )
    moon_vec = np.array([-0.5168773177603934, -0.5735381958030377, 0.6355248038745751])
    sun_vec = np.array([0.6715534288821232, -0.7248213943012205, -0.1537853651031126])
    cameraMeasurements1 = CameraMeasurementVector(
        angular_separation(earth_vec, moon_vec),
        angular_separation(earth_vec, sun_vec),
        angular_separation(moon_vec, sun_vec),
        0.18657584471242206,
        0.008208124935221156,
        0.009151425262957794,
    )

    # Data to test mean_squared_error
    MSE_Test_State_vector_1 = TrajectoryStateVector(
        1.433e05, 5.3541e05, 1.9355e05, -0.93198, -0.077729, 0.049492
    )
    MSE_Test_State_vector_2 = TrajectoryStateVector(
        1.433e05, 5.3541e05, 1.9355e05, -0.93198, -0.077729, 0.049492
    )

    """###################### end test data ################################"""

    def MSE_test(self, MSE_Test_State_vector_1, MSE_Test_State_vector_2):
        # Check that MSE for position is 0
        assert (
            MSE(MSE_Test_State_vector_1, MSE_Test_State_vector_2)[0] == 0
        ), "Incorrect Mean Squared Error Computation for position"

        # Check that MSE for velocity is 0
        assert (
            MSE(MSE_Test_State_vector_1, MSE_Test_State_vector_2)[1] == 0
        ), "Incorrect Mean Squared Error Computation for velocity"

    def get_angular_seperations(time):
        with open("observations.json", "r") as data:
            json_object = json.load(data)

        mylst = []

        item = json_object["observations"]
        for times in item:
            if times["time"] == time:
                target = times["observed_bodies"]
                for obj in target:
                    mylst.append(obj["direction_body"])
                break
        return mylst

    tests_3600_dt = []

    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))

    with open("traj2.csv", "r") as data:
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
        tests_3600_dt.append(
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
        assert posError <= 1000, "Position error is too large"
        assert velError <= 5, "Velocity error is too large"

        # assert np.all(np.less_equal(lower, trajEstimateOutput.new_state.data.T))
        # assert np.all(np.less_equal(trajEstimateOutput.new_state.data.T, upper))

    def test_ignore(self):
        trajStateVector = TrajectoryStateVector(
            -2.7699842997503623e4,
            -1.4293835289027357e4,
            1713.3139597599213,
            -2.8869995362010077,
            -3.7832739597202594,
            -0.9225267990120935,
        )
        main_thrust_info = None
        dynamicsOnly = True
        moonEph = EphemerisVector(
            3.688288588957627e5, 1.4820784006906873e4, -3.0687755261273645e4, 0, 0, 0
        )
        sunEph = EphemerisVector(
            1.6896327218153402e7, -1.3867258359275745e8, -6.011476692460759e7, 0, 0, 0
        )
        cameraMeasurements = None
        P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float))

        trajEstimateOutput = runTrajUKF(
            moonEph,
            sunEph,
            cameraMeasurements,
            trajStateVector,
            208800,
            CovarianceMatrix(P),
            main_thrust_info,
            dynamicsOnly,
        )
        trajNew = trajEstimateOutput.new_state.data
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
        # for some reason the calculations below do not match with the
        # calculations above:
        # posError = math.sqrt( np.sum((trajNew[:3] - truthState[:3])**2) )
        # velError = math.sqrt( np.sum((trajNew[3:6] - truthState[3:6])**2) )
        logging.debug(f"Output x position: {trajNew[0]}\n")
        logging.debug(f"Output y position: {trajNew[1]}\n")
        logging.debug(f"Output z position: {trajNew[2]}\n")
        logging.debug(f"Position error: {posError}\nVelocity error: {velError}")
        assert posError <= 1000, "Position error is too large"
        assert velError <= 5, "Velocity error is too large"

    def test_iterative_nontruth(self):
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
        df = pd.read_csv("traj2.csv")

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
        logging.debug(f"Position error: {posError}\nVelocity error: {velError}\n")
        assert posError <= 1000, "Position error is too large"
        assert velError <= 5, "Velocity error is too large"


if __name__ == "__main__":
    unittest.main()
