import numpy as np
import unittest
import os
import json
import re


from core.const import FileData, DetectionData, Vector3
from core.observe_functions import cam_to_body, body_to_T0
from utils.constants import FLIGHT_SOFTWARE_PATH


class BodyMeas(unittest.TestCase):
    def st_to_sph(self, x, y):
        """Convert stereographic coordinates to spherical coordinates."""
        norm = x ** 2 + y ** 2 + 4
        # Added a negative sign to z value below
        return 4 * x / norm, 4 * y / norm, -(norm - 8) / norm

    def get_data(self, path):
        data = open(path)
        obs = json.load(data)
        frames = obs["observations"][0]["frames"]
        fileInfo = []
        centerSt = []
        dt = []
        truthT0Vec = []
        for frame in frames:

            centerSt.append(frame["detections"][0]["center_st"])

            imgName = frame["image_gnomonic"]
            fileInfo.append(FileData(imgName))

            # Can't use timestamp in fileInfo b/c that gets incorrectly parsed with file name
            # convention of opnav sim (timestamps have a decimal point)
            dtFrame = float(re.search(r"[dt](\d*\.?\d+)", imgName).group(1))
            dt.append(dtFrame)

            body = frame["detections"][0]["body"]
            bodyNum = 0 if body == "Earth" else 1 if body == "Moon" else 2
            truthT0Vec.append(
                obs["observations"][0]["observed_bodies"][bodyNum]["direction_body"]
            )

        gyroY = obs["observations"][0]["spacecraft"]["omega_body"][1]

        data.close()
        return fileInfo, centerSt, dt, truthT0Vec, gyroY

    def body_meas(self, path):
        fileInfo, centerSt, dt, truthT0Vec, gyroY = self.get_data(path)

        print(f"\n{path}")
        for i in range(len(fileInfo)):
            camNum = fileInfo[i].cam_num
            print("CamNum: ", camNum)
            print("center_st: ", centerSt[i])
            camVec = self.st_to_sph(centerSt[i][0], centerSt[i][1])
            print("Cam Vec: ", camVec)

            detection = DetectionData(
                filedata=fileInfo[i],
                vector=Vector3(camVec[0], camVec[1], camVec[2]),
                ang_diam=None,
                detection=None,
            )

            # Camera frame to satellite body frame
            bodyDet = cam_to_body(detection)
            print("Satellite Frame Vector: ", bodyDet.vector)

            # Satellite body frame to T0 frame
            finalT0Det = body_to_T0(bodyDet, dt[i], gyroY)
            print("Observe Start Vector: ", finalT0Det.vector)

            print("Actual Vector: ", truthT0Vec[i])

            vecDist = np.linalg.norm(finalT0Det.vector.data - truthT0Vec[i])
            print("Vect Dist: ", vecDist)
            self.assertLessEqual(
                vecDist,
                0.05,
                "Body transformations do not match within margin of error!",
            )
            print()

    def test_traj_case1c(self):
        path = os.path.join(
            FLIGHT_SOFTWARE_PATH,
            "OpticalNavigation/simulations/sim/data/traj-case1c_sim/observations.json",
        )
        self.body_meas(path)

    def test_traj_trajectory(self):
        path = os.path.join(
            FLIGHT_SOFTWARE_PATH,
            "OpticalNavigation/simulations/sim/data/trajectory_sim/observations.json",
        )
        self.body_meas(path)


if __name__ == "__main__":
    unittest.main()
