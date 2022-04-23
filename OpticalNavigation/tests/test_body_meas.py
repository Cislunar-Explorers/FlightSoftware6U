import numpy as np
import unittest
import os
import json
import re
import logging

from core.find_algos.tiled_remap import st_to_sph
from core.const import FileData, DetectionData, Vector3
from core.observe_functions import cam_to_body, body_to_T0
from core.opnav import calculate_cam_measurements
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
            # radiusSt = []
            dt = []
            truthT0Vec = []
            truthT0Size = []
            for frame in sorted(frames, key=lambda f: f["image_gnomonic"]):
                imgName = frame["image_gnomonic"]
                fileInfo.append(FileData(imgName))

                centerSt.append(frame["detections"][0]["center_st"])

                # radiusSt.append(frame["detections"][0]["radius_st"])

                # Can't use timestamp in fileInfo b/c that gets incorrectly parsed with file name
                # convention of opnav sim (timestamps have a decimal point)
                dtFrame = float(re.search(r"[dt](\d*\.?\d+)", imgName).group(1))
                dt.append(dtFrame)

                body = frame["detections"][0]["body"]
                bodyNum = 0 if body == "Earth" else 1 if body == "Moon" else 2
                truthT0Vec.append(
                    obs["observations"][0]["observed_bodies"][bodyNum]["direction_body"]
                )
                truthT0Size.append(
                    obs["observations"][0]["observed_bodies"][bodyNum]["angular_size"]
                )

            gyroY = obs["observations"][0]["spacecraft"]["omega_body"][1]
        return fileInfo, centerSt, dt, truthT0Vec, truthT0Size, gyroY

    def body_meas(self, fileInfo, centerSt, dt, truthT0Vec, gyroY):
        calc_vecs = []
        truth_vecs = []
        errors = []
        for (fileInf, stCenter, deltaT, truthVec) in zip(
            fileInfo, centerSt, dt, truthT0Vec
        ):
            logging.debug(f"Filename: {fileInf.filename}")
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

            logging.debug(f"Actual Vector: {truthVec}")

            # We are comparing unit vectors here
            vectSep = calculate_cam_measurements(finalT0Det.vector.data, truthVec)
            logging.debug(f"Vector Angular Separation: {vectSep}")

            diffVec = finalT0Det.vector.data - truthVec
            logging.debug(f"Difference Vector: {diffVec}")
            vecDist = np.linalg.norm(diffVec)
            logging.debug(f"Vect Dist: {vecDist}")
            self.assertLessEqual(
                vecDist,
                0.05,
                "Body transformations do not match within margin of error!",
            )

            # dot = np.dot(finalT0Det.vector.data, truthVec)
            # truthNorm = np.linalg.norm(truthVec)
            # calcNorm = np.linalg.norm(finalT0Det.vector.data)
            # vAngle = np.arccos(dot / (truthNorm * calcNorm))

            # logging.debug(f"Angle Dist: {vAngle} (rad) = {vAngle * 180 / np.pi} (deg)")
            # logging.debug(f"Radial Dist: {truthNorm - calcNorm}")
            logging.debug("\n")
            calc_vecs.append(finalT0Det.vector.data)
            truth_vecs.append(truthVec)
            errors.append(vecDist)
        calc_vecs = zip(calc_vecs, fileInfo)
        truth_vecs = zip(truth_vecs, fileInfo)
        errors = zip(errors, fileInfo)

        return calc_vecs, truth_vecs, errors

    def test_traj_case1c(self):
        path = os.path.join(
            FLIGHT_SOFTWARE_PATH,
            "OpticalNavigation/simulations/sim/data/traj-case1c_sim_no_outline/observations.json",
        )
        fileInfo, centerSt, dt, truthT0Vec, _, gyroY = self.get_data(path)
        _, _, _ = self.body_meas(fileInfo, centerSt, dt, truthT0Vec, gyroY)
        for i in fileInfo:
            logging.debug(i)

    # def test_traj_trajectory(self):
    # path = os.path.join(
    # FLIGHT_SOFTWARE_PATH,
    # "OpticalNavigation/simulations/sim/data/trajectory_sim/observations.json",
    # )
    # fileInfo, centerSt, radiusSt, dt, truthT0Vec, truthT0Size, gyroY = self.get_data(path)
    # _, _, _ = self.body_meas(fileInfo, centerSt, radiusSt, dt, truthT0Vec, truthT0Size, gyroY)


if __name__ == "__main__":
    unittest.main()
