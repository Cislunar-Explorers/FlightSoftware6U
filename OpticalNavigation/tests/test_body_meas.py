import numpy as np
import unittest
import os
import json
import re


from core.const import CisLunarCameraParameters
from core.observe_functions import tZeroRotMatrix
from utils.constants import FLIGHT_SOFTWARE_PATH


class BodyMeas(unittest.TestCase):
    def st_to_sph(self, x, y):
        """Convert stereographic coordinates to spherical coordinates."""
        norm = x ** 2 + y ** 2 + 4
        # Added a negative sign to z value below
        return 4 * x / norm, 4 * y / norm, -(norm - 8) / norm

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
            return np.array([0, 0, 0])

    def body2T0(self, bodyVec, gyroY, timeElapsed) -> np.ndarray:
        bodyRotation = gyroY * timeElapsed
        T0RotMatrix = tZeroRotMatrix(bodyRotation)
        bodyT0 = np.dot(T0RotMatrix, bodyVec)
        return bodyT0

    def get_traj_case_1c_data(self):
        path = os.path.join(
            FLIGHT_SOFTWARE_PATH,
            "OpticalNavigation/simulations/sim/data/traj-case1c_sim/observations.json",
        )
        data = open(path)
        obs = json.load(data)
        frames = obs["observations"][0]["frames"]  # Need to iterate over all frames
        camNum = []
        centerSt = []
        dt = []
        truthT0Vec = []
        for frame in frames:
            cam = frame["camera"]
            cam = 1 if cam == "A" else 2 if cam == "B" else 3
            camNum.append(cam)

            centerSt.append(frame["detections"][0]["center_st"])

            imgName = frame["image_gnomonic"]
            dtFrame = float(re.search(r"[dt](\d*\.?\d+)", imgName).group(1))
            dt.append(dtFrame)

            body = frame["detections"][0]["body"]
            bodyNum = 0 if body == "Earth" else 1 if body == "Moon" else 2
            truthT0Vec.append(
                obs["observations"][0]["observed_bodies"][bodyNum]["direction_body"]
            )  # Depends on body

        gyroY = obs["observations"][0]["spacecraft"]["omega_body"][1]

        data.close()
        return camNum, centerSt, dt, truthT0Vec, gyroY

    def test_body_meas(self):
        camNum, centerSt, dt, truthT0Vec, gyroY = self.get_traj_case_1c_data()
        print("Camera Numbers: ", camNum)

        for i in range(len(camNum)):
            print("CamNun: ", camNum[i])
            print(i)
            print("center_st: ", centerSt[i])
            camVec = self.st_to_sph(centerSt[i][0], centerSt[i][1])
            print("Cam Vec: ", camVec)

            # Camera frame to satellite body frame
            bodyVec = self.cam2body(camVec, camNum[i])
            print("Satellite Frame Vector: ", bodyVec)

            # Satellite body frame to T0 frame
            print("dt: ", dt[i])
            finalT0Vec = self.body2T0(bodyVec, gyroY, dt[i])
            print("Observe Start Vector: ", finalT0Vec)

            print("Actual Vector: ", truthT0Vec[i])

            vecDist = np.linalg.norm(finalT0Vec - truthT0Vec[i])
            print("Vect Dist: ", vecDist)
            self.assertLessEqual(
                vecDist,
                0.05,
                "Body transformations do not match within margin of error!",
            )
            print()


if __name__ == "__main__":
    unittest.main()
    # a = BodyMeas()
    # a.get_traj_case_1c_data()
