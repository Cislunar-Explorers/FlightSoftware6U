from core.observe_functions import get_ephemeris
from core.const import (
    BodyEnum,
    CovarianceMatrix,
    EphemerisVector,
    TrajectoryStateVector,
)
import numpy as np
from core.ukf import runTrajUKF


"""def runTrajUKF(moonEph: EphemerisVector, sunEph: EphemerisVector, measurements:CameraMeasurementVector,
            initState:TrajectoryStateVector, dt:np.float, P:CovarianceMatrix, cameraParams:CameraParameters,
            main_thrust_info:MainThrustInfo=None, dynamicsOnly:bool=False) -> TrajectoryEstimateOutput:"""


class trivialTestUKF:

    # Trivial test for UKF implementation (Velocities and positions are all 0 for easy calculations)
    def test_trivial_ukf():

        moonEph = get_ephemeris(0, BodyEnum.Moon)  # Moon ephemeris data from astro.py
        moonEph = EphemerisVector(
            moonEph[0], moonEph[1], moonEph[2], moonEph[3], moonEph[4], moonEph[5]
        )

        sunEph = get_ephemeris(0, BodyEnum.Sun)  # Sun ephemeris data from astro.py
        sunEph = EphemerisVector(
            sunEph[0], sunEph[1], sunEph[2], sunEph[3], sunEph[4], sunEph[5]
        )

        trajStateVector = TrajectoryStateVector(
            100000, 10000, 10000, 0.1, 0.1, 0.1
        )  # positions and velocities are all 0
        dt = 0.01  # Step size with respect to time
        P = np.diag(
            np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=float)
        )  # Initial covariance matrix
        measurements = None  # If None gets initialized to 1x6 zero tuple
        cameraParams = None  # Does not get used in code
        trajEstimateOutput = runTrajUKF(
            moonEph,
            sunEph,
            measurements,
            trajStateVector,
            dt,
            CovarianceMatrix(P),
            cameraParams,
        )
        # has new_state : TrajectoryStateVector, new_P : CovarianceMatrix, K: Matrix6x6 (K = Kalman Gain)

        # expected outputs for state (trajStateVector)
        # one way to test outputs are correct is to separately call functions from ukf.py:
        """expected_xpos = 0
        expected_ypos = 0
        expected_zpos = 0
        expected_xvel = 0
        expected_yvel = 0
        expected_zvel = 0"""  # Commented out for commit

        pos_array = trajEstimateOutput.new_state.get_position_data()
        vel_array = trajEstimateOutput.new_state.get_velocity_data()
        print("x_pos = " + str(pos_array[0]))
        print("y_pos = " + str(pos_array[1]))
        print("z_pos = " + str(pos_array[2]))
        print("x_vel = " + str(vel_array[0]))
        print("y_vel = " + str(vel_array[1]))
        print("z_vel = " + str(vel_array[2]))

        # expected output for covariance matrix
        """expected_P = None"""  # Commented out for commit
        print("out covariance = ")
        print(str(trajEstimateOutput.new_P))
        # expected kalman gain
        """expected_K = None"""  # Commented out for commit
        print("out kalman = ")
        print(str(trajEstimateOutput.K))


a = trivialTestUKF
a.test_trivial_ukf()
