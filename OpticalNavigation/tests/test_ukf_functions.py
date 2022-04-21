from core.const import (
    CameraMeasurementVector,
    CovarianceMatrix,
    EphemerisVector,
    TrajectoryStateVector,
)
import numpy as np

from core.ukf import runTrajUKF
import unittest
from parameterized import parameterized
import logging
import json

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

    """###################### end test data ################################"""

    def get_angular_seperations(time):
        json_object = json.load(open("observations.json"))

        mylst = []

        item = json_object["observations"]
        for times in item:
            if times["time"] == time:
                target = times["observed_bodies"]
                for obj in target:
                    mylst.append(obj["direction_body"])
                break
        return mylst

    @parameterized.expand(
        [
            [moonEph1, sunEph1, trajStateVector1, dt1, P1, None],
            [moonEph2, sunEph2, trajStateVector2, dt2, P2, cameraMeasurements1],
        ]
    )
    def test_sequence(
        self, moonEph, sunEph, trajStateVector, dt, P, cameraMeasurements
    ):
        logging.debug("**********************************\n")

        main_thrust_info = None
        dynmaicsOnly = True
        trajEstimateOutput = runTrajUKF(
            moonEph,
            sunEph,
            cameraMeasurements,
            trajStateVector,
            dt,
            CovarianceMatrix(P),
            main_thrust_info,
            dynmaicsOnly,
        )
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
        deviation = np.matmul(P, np.ones(6).T)  # 6 x 1 col vector
        upper = np.add(deviation, trajStateVector.data.T)
        lower = np.subtract(trajStateVector.data.T, deviation)

        assert np.all(np.less_equal(lower, trajEstimateOutput.new_state.data.T))
        assert np.all(np.less_equal(trajEstimateOutput.new_state.data.T, upper))


if __name__ == "__main__":
    unittest.main()
