import numpy as np
import unittest
import os
import json
import re


from core.const import CisLunarCameraParameters
from core.opnav import _tZeroRotMatrix
from utils.constants import FLIGHT_SOFTWARE_PATH

class BodyMeas(unittest.TestCase):
    
    def cam2body(self, camVec, camNum) -> np.ndarray:
        cam_params = CisLunarCameraParameters
        if camNum == 1:
            return cam_params.cam1Rotation.dot(camVec)
        elif camNum == 2:
            return cam_params.cam2Rotation.dot(camVec)
        elif camNum == 3:
            return cam_params.cam3Rotation.dot(camVec)
        else:
            print("Error")
            return np.array([0,0,0])
    
    def body2T0(self, bodyVec, gyroY, timeElapsed) -> np.ndarray:
        bodyRotation = gyroY * timeElapsed
        T0RotMatrix = _tZeroRotMatrix(bodyRotation)
        bodyT0 = np.dot(T0RotMatrix, bodyVec)
        return bodyT0

    def get_traj_case_1c_data(self):
        path = os.path.join(FLIGHT_SOFTWARE_PATH, "OpticalNavigation/simulations/sim/data/traj-case1c_sim/observations.json")
        data = open(path)
        obs = json.load(data)
        frames = obs["observations"][0]["frames"][0] # Need to iterate over all frames
        
        camNum = frames["camera"]
        camNum = 1 if camNum == "A" else 2 if camNum == "B" else 3
        
        camVec = frames["detections"][0]["direction_cam"]
        
        img_name = frames["image_gnomonic"]
        dt = float(re.search(r"[dt](\d*\.?\d+)", img_name).group(1))
        
        gyroY = obs["observations"][0]["spacecraft"]["omega_body"][1]
        
        truthT0Vec = obs["observations"][0]["observed_bodies"][2]["direction_body"]
        data.close()
        return camNum, camVec, dt, gyroY, truthT0Vec
        
    def test_body_meas(self):
        camNum, camVec, dt, gyroY, truthT0Vec = self.get_traj_case_1c_data()
        print("Camera Number: ", camNum)
        print("Camera Vector: ", camVec)
        # Need to rotate camVec 180dg about z axis because of differences in how camera frame is defined in sim vs fsw
        # This allows for our transofmration functions to work, we end up in the same T0 frame
        camVec[0] = -1 * camVec[0]
        camVec[1] = -1 * camVec[1]
        # Camera frame to satellite body frame
        bodyVec = self.cam2body(camVec, camNum)
        print("Satellite Frame Vector: ", bodyVec)

        # Satellite body frame to T0 frame
        finalT0Vec = self.body2T0(bodyVec, gyroY, timeElapsed=dt)
        print("Observe Start Vector: ", finalT0Vec)

        print("Actual Vector: ", truthT0Vec)

        vecDist = np.linalg.norm(finalT0Vec - truthT0Vec)
        print("Vect Dist: ", vecDist)
        self.assertLessEqual(vecDist, 0.05, "Body transformations do not match within margin of error!")
        


if __name__ == '__main__':
    unittest.main()
    #BodyMeas.get_traj_case_1c_data()