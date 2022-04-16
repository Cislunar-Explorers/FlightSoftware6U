from core.const import CovarianceMatrix, EphemerisVector, TrajectoryStateVector
import numpy as np
from core.ukf import runTrajUKF
import unittest
from parameterized import parameterized

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


class TestSequence(unittest.TestCase):

    """
    to add more tests:
    add moonEph, sunEph, trajStateVector
    add [moonEph, sunEph, trajStateVector, dt, P] in @parameterized.expand
    """

    """###################### begin test data ################################"""

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

    moonEph2 = EphemerisVector(
        1.5363e05, -3.7237e05, 2887.6, 0.90889, 0.34863, -0.088026
    )
    sunEph2 = EphemerisVector(
        -3.0672e07, -1.441e08, 6670.4, 29.633, -6.0859, -0.00088015
    )
    trajStateVector2 = TrajectoryStateVector(
        1.433e05, 5.3541e05, 1.9355e05, -0.93198, -0.077729, 0.049492
    )

    """###################### end test data ################################"""

    @parameterized.expand(
        [
            [moonEph1, sunEph1, trajStateVector1, dt1, P1],
            [moonEph2, sunEph2, trajStateVector2, dt1, P1],
        ]
    )
    def test_sequence(self, moonEph, sunEph, trajStateVector, dt, P):
        print("**********************************\n")
        cameraMeasurements = None
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
        print("x,y,z position = ", pos_array)
        print("x,y,z velocity = ", vel_array)
        # expected output for covariance matrix
        print("out covariance = \n", trajEstimateOutput.new_P)
        # kalman gain is all 0s -->
        print("out kalman = \n", trajEstimateOutput.K)
        print("\n")

        # validate output
        deviation = np.matmul(P, np.ones(6).T)  # 6 x 1 col vector
        upper = np.add(deviation, trajStateVector.data.T)
        lower = np.subtract(trajStateVector.data.T, deviation)

        assert np.all(np.less_equal(lower, trajEstimateOutput.new_state.data.T))
        assert np.all(np.less_equal(trajEstimateOutput.new_state.data.T, upper))


if __name__ == "__main__":
    unittest.main()
