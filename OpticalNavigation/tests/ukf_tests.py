from unittest import TestCase
import pandas as pd
import numpy as np
import math
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory
from tests.const import TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph, TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph
from core.ukf import runUKF

class TestFindOnEclipseAndCrescentImagesDataset(TestCase):

    def __init__(self, *args, **kwargs):
        super(TestFindOnEclipseAndCrescentImagesDataset, self).__init__(*args, **kwargs)
        self.POS_ERROR = 1000 # can be off by 1000 km
        self.VEL_ERROR = 1 # can be off by 1 km/s

    def test_dynamics_model_EM1_3DOF_Trajectory_June_27_2020_3600sec(self):
        """
        Assumes first state vector is the initial state provided by NASA at the start of mission.
        Each run is treated as a 're-initialization' of the UKF, meaning that the delta time of 3600 secs is considered
        'too long' for the UKF to converge properly.
        Because we don't have access to the cam_meas functions, the Kalman Gain is set to 0, which means it is testing the dynamics model only.
        Tests:
        - expected state is close to actual state
        - 
        """
        trajTruthdf = pd.read_csv(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec + TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_trajectory)
        moonEphdf = pd.read_csv(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec + TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_moonEph)
        sunEphdf = pd.read_csv(TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec + TEST_EM1_3DOF_Trajectory_June_27_2020_3600sec_sunEph)
        state = (np.array([trajTruthdf.iloc[0]['x'], trajTruthdf.iloc[0]['y'], trajTruthdf.iloc[0]['z'], trajTruthdf.iloc[0]['vx'], trajTruthdf.iloc[0]['vy'], trajTruthdf.iloc[0]['vz']], dtype=np.float)).reshape(6,1)
        for t in range(trajTruthdf.shape[0] - 1):
            moonEph = (np.array([moonEphdf.iloc[t]['x'], moonEphdf.iloc[t]['y'], moonEphdf.iloc[t]['z'], moonEphdf.iloc[t]['vx'], moonEphdf.iloc[t]['vy'], moonEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
            sunEph = (np.array([sunEphdf.iloc[t]['x'], sunEphdf.iloc[t]['y'], sunEphdf.iloc[t]['z'], sunEphdf.iloc[t]['vx'], sunEphdf.iloc[t]['vy'], sunEphdf.iloc[t]['vz']], dtype=np.float)).reshape(1,6)
            P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
            state, pNew, K = runUKF(moonEph, sunEph, np.zeros_like(state), state, 60, P, dynamicsOnly=True)

        t = trajTruthdf.shape[0] - 1
        traj = (np.array([trajTruthdf.iloc[t]['x'], trajTruthdf.iloc[t]['y'], trajTruthdf.iloc[t]['z'], trajTruthdf.iloc[t]['vx'], trajTruthdf.iloc[t]['vy'], trajTruthdf.iloc[t]['vz']], dtype=np.float)).reshape(6,1)
        print(state, traj)
            
        traj = traj.flatten()
        state = state.flatten()
        posError = math.sqrt( np.sum((traj[:3] - state[:3])**2) )
        velError = math.sqrt( np.sum((traj[3:6] - state[3:6])**2) )
        print(posError, velError)
        self.assertLessEqual(posError, self.POS_ERROR)
        self.assertLessEqual(velError, self.VEL_ERROR)
        self.fail('Test implementation in progress...')

if __name__ == '__main__':
    unittest.main()