import numpy as np
import unittest
import os
import json
import re
import logging

from core.find_algos.tiled_remap import st_to_sph
from core.const import FileData, DetectionData, Vector3
from core.observe_functions import cam_to_body, body_to_T0
from utils.constants import FLIGHT_SOFTWARE_PATH


class BodyMeas(unittest.TestCase):
    """
    Tests the correctness of the tranformations used in taking a sterographic coordinate in the camera frame to a
    3D xyz unit vector in the satellite's body frame at the start of acquisition. This test achieves this by using the
    truth data in the opnav sim's observations.json files as reference. The stereographic coordinate is taken as input
    from the json and is taken through the sequence of transformations. The final result is compared with the final
    truth value from the json.
    """

    def get_data(self, path):
        with open(path, "r") as data:
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
        return fileInfo, centerSt, dt, truthT0Vec, gyroY

    def body_meas(self, path):
        fileInfo, centerSt, dt, truthT0Vec, gyroY = self.get_data(path)

        logging.debug(f"{path}")

        for (fileInf, stCenter, deltaT, truth) in zip(
            fileInfo, centerSt, dt, truthT0Vec
        ):
            camNum = fileInf.cam_num
            logging.debug(f"CamNum: {camNum}")
            logging.debug(f"Center_st: {stCenter}")
            camVec = st_to_sph(stCenter[0], stCenter[1])
            logging.debug(f"Cam Vec: {camVec}")

            detection = DetectionData(
                filedata=fileInf,
                vector=Vector3(camVec[0], camVec[1], camVec[2]),
                ang_diam=None,
                detection=None,
            )

            # Camera frame to satellite body frame
            bodyDet = cam_to_body(detection)
            logging.debug(f"Satellite Frame Vector: {bodyDet.vector}")

            # Satellite body frame to T0 frame
            finalT0Det = body_to_T0(bodyDet, deltaT, gyroY)
            logging.debug(f"Observe Start Vector: {finalT0Det.vector}")

            logging.debug(f"Actual Vector: {truth}")

            # We are comparing unit vectors here
            vecDist = np.linalg.norm(finalT0Det.vector.data - truth)
            logging.debug(f"Vect Dist: {vecDist}")
            self.assertLessEqual(
                vecDist,
                0.05,
                "Body transformations do not match within margin of error!",
            )
            logging.debug("\n")

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
