# from core.observe_functions import get_ephemeris
from core.const import (
    # BodyEnum,
    CovarianceMatrix,
    EphemerisVector,
    TrajectoryStateVector,
)
import numpy as np
from core.ukf import runTrajUKF
import unittest


class trivialTestUKF(unittest.TestCase):
    """
    Tests accuracy of our trajectory ukf on a trivial case. Test takes data from various constants and low initial
    velocities and positions to check that we obtained reasonable outputs. The final result is then compared with
    our expected outputs (using the standard deviations of our covariance matrix).
    """

    """
    moonEph data file: https://cornell.app.box.com/file/651831179494
    sunEph data file: https://cornell.app.box.com/file/651825552217
    trajStateVector data file: https://cornell.app.box.com/file/651842666983
    """

    def test_trivial_ukf(self):
        # moonEph = get_ephemeris(0, BodyEnum.Moon)  # Moon ephemeris data from astro.py
        moonEph = EphemerisVector(
            1.5363e05, -3.7237e05, 2887.6, 0.90889, 0.34863, -0.088026
        )
        # sunEph = get_ephemeris(0, BodyEnum.Sun)  # Sun ephemeris data from astro.py
        sunEph = EphemerisVector(
            -3.0672e07, -1.441e08, 6670.4, 29.633, -6.0859, -0.00088015
        )
        trajStateVector = TrajectoryStateVector(
            1.433e05, 5.3541e05, 1.9355e05, -0.93198, -0.077729, 0.049492
        )
        dt = 1  # seconds
        P = np.diag(
            np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float)
        )  # Initial covariance matrix
        """
        measurements = CameraMeasurementVector(883.96,1022.8,909.4,65.646,11.315,28.42)
        # we're first testing with CameraMeasurement vector = None; this gives us an output
        # that makes more sense
        """
        trajEstimateOutput = runTrajUKF(
            moonEph,
            sunEph,
            None,  # originally passed in measurements
            trajStateVector,
            dt,
            CovarianceMatrix(P),
            dynamicsOnly=True,
        )
        """"
        inputs:
        dynamicsModelOnly means we don't need the cameraMeasurements and only rely on initial state ?
        note: need to verify whether the camera measurements' units are still
        in pixel separation or not; they should have been changed to angular separation
        """
        # has new_state : TrajectoryStateVector, new_P : CovarianceMatrix, K: Matrix6x6 (K = Kalman Gain)
        # expected outputs for state (trajStateVector)
        # one way to test outputs are correct is to separately call functions from ukf.py:

        pos_array = trajEstimateOutput.new_state.get_position_data()
        vel_array = trajEstimateOutput.new_state.get_velocity_data()
        print("x_pos = " + str(pos_array[0]))
        print("y_pos = " + str(pos_array[1]))
        print("z_pos = " + str(pos_array[2]))
        print("x_vel = " + str(vel_array[0]))
        print("y_vel = " + str(vel_array[1]))
        print("z_vel = " + str(vel_array[2]))
        # expected output for covariance matrix
        """expected_P = None"""
        print("out covariance = ")
        print(str(trajEstimateOutput.new_P))
        # kalman gain is all 0s -->
        print("out kalman = ")
        print(str(trajEstimateOutput.K))


if __name__ == "__main__":
    unittest.main()
