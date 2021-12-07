import numpy as np
import unittest
import os
import json
import re

from core.const import CisLunarCameraParameters
from core.opnav import _tZeroRotMatrix
from utils.constants import FLIGHT_SOFTWARE_PATH

class BodyMeas(unittest.TestCase):

    cam_params = CisLunarCameraParameters
    def cam2body(self, camVec, camNum) -> np.ndarray:
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


    def get_traj_case_1c_data():
        path = os.path.join(FLIGHT_SOFTWARE_PATH, "OpticalNavigation/simulations/sim/data/traj-case1c_sim/observations.json")
        data = open(path)
        obs = json.load(data)
        frames = obs["observations"][0]["frames"][0]
        camNum = frames["camera"]
        camNum = 1 if camNum == "A" else 2 if camNum == "B" else 3
        print(camNum)
        camVec = frames["detections"][0]["direction_cam"]
        print(camVec)
        img_name = frames["image_gnomonic"]
        dt = float(re.search(r"[dt](\d*\.?\d+)", img_name).group(1))
        print(dt)
        
    def test_body_meas(self):
        '''
        # Get params from json
        camVec = json.get("direction_cam")
        camNum = json.get("camera".convert to [1, 2, 3])
        dt = json.get(parse filename for dt)
        gyroY = json.get("omega_body"[1])
        truthT0Vec = json.get("observed_bodies"->"body"->"direction_body")
        
        # Camera frame to satellite body frame
        bodyVec = cam2body(camVec, camNum)

        # Satellite body frame to T0 frame
        finalT0Vec = body2T0(bodyVec, gyroY, timeElapsed=dt)

        vecDist = np.linalg.norm(finalT0Vec - truthT0Vec)
        accuracy = math.abs((finalT0Vec - truthT0Vec)/truthT0Vec)
        self.assertLessEqual(accuracy, 0.05, "Body transformations do not match within margin of error!")
        '''


if __name__ == '__main__':
    #unittest.main()
    BodyMeas.get_traj_case_1c_data()