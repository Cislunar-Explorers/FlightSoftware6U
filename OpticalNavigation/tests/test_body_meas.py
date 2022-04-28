# import numpy as np
import unittest
import os
import json
import re
import logging

from core.find_algos.tiled_remap import st_to_sph
from core.const import BodyEnum, FileData, DetectionData, Vector3
from core.observe_functions import cam_to_body, body_to_T0
from core.opnav import calculate_cam_measurements
from utils.constants import FLIGHT_SOFTWARE_PATH


class jsonData:
    def __init__(self, filename) -> None:
        pass


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

    def check(self, calcVec, truthVec):
        angSep = calculate_cam_measurements(calcVec, truthVec)
        return angSep

    # Compares transforming sim truth stereographic coordinates with truth sim vector
    def sim_transform(self, stVecs, truthVecs, gyroY):
        for f in stVecs.keys():
            logging.debug(f"File: {f}")
            fileInfo = FileData(f)
            dt = float(re.search(r"[dt](\d*\.?\d+)", f).group(1))
            for body in (BodyEnum.Earth, BodyEnum.Moon, BodyEnum.Sun):
                if body in stVecs[f].keys():
                    camVec = st_to_sph(stVecs[f][body][0], stVecs[f][body][1])
                    logging.debug(
                        f"Cam Vec: [{stVecs[f][body][0]}, {stVecs[f][body][1]}]"
                    )
                    detection = DetectionData(
                        filedata=fileInfo,
                        vector=Vector3(camVec[0], camVec[1], camVec[2]),
                        ang_diam=None,
                        detection=None,
                    )
                    logging.debug(f"{fileInfo.cam_num=}")
                    # Camera frame to satellite body frame
                    bodyDet = cam_to_body(detection)
                    # logging.debug(f"Satellite Frame Vector: {bodyDet.vector}")

                    # Satellite body frame to T0 frame
                    finalT0Det = body_to_T0(bodyDet, dt, gyroY)
                    logging.debug(f"Observe Start Vector: {finalT0Det.vector}")
                    logging.debug(f"Truth Vector: {truthVecs[f][body]}")
                    angSep = calculate_cam_measurements(
                        finalT0Det.vector.data, truthVecs[f][body]
                    )
                    logging.debug(f"{angSep=}\n")

        """
        for (fileInf, stCenter, deltaT) in zip(fileInfo, centerSt, dt):
            logging.debug(f"Filename: {fileInf.filename}")
            camNum = fileInf.cam_num
            logging.debug(f"CamNum: {camNum}")

            # Stereographic coordinate to spherical coordinate
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
            logging.debug(f"Observe Start Vector: {finalT0Det.vector}\n")

            calc_vecs.append(detection)
        # calc_vecs = zip(calc_vecs, fileInfo)

        return calc_vecs
        """

    def load_json(self, jsonPath):
        with open(jsonPath, "r") as data:
            obs = json.load(data)
            frames = obs["observations"][0]["frames"]
            st_dict = {}
            sph_dict = {}
            gyroY = obs["observations"][0]["spacecraft"]["omega_body"][1]
            for frame in sorted(frames, key=lambda f: f["image_stereographic"]):
                imgName = frame["image_stereographic"]
                # fileInfo = FileData(imgName)
                # dtFrame = float(re.search(r"[dt](\d*\.?\d+)", imgName).group(1))
                # logging.debug(f"{imgName=}")
                # logging.debug(frame["detections"])
                det_st_dict = {}
                det_sph_dict = {}
                for det in frame["detections"]:
                    body = det["body"]
                    body = (
                        BodyEnum.Earth
                        if body == "Earth"
                        else BodyEnum.Moon
                        if body == "Moon"
                        else BodyEnum.Sun
                    )
                    det_st_dict[body] = det["center_st"]
                    det_sph_dict[body] = det["direction_cam"]
                st_dict[imgName] = det_st_dict
                sph_dict[imgName] = det_sph_dict
            logging.debug(st_dict)
            logging.debug(sph_dict)

            # Get truth sizes
            diam_dict = {}
            for b in (BodyEnum.Earth, BodyEnum.Moon, BodyEnum.Sun):
                ang_diam = obs["observations"][0]["observed_bodies"][b]["angular_size"]
                diam_dict[b] = ang_diam

            gyroY = obs["observations"][0]["spacecraft"]["omega_body"][1]

            logging.debug(diam_dict)

        return st_dict, sph_dict, diam_dict, gyroY

    def test_case1c(self):
        path = os.path.join(
            FLIGHT_SOFTWARE_PATH,
            "OpticalNavigation/simulations/sim/data/traj-case1c_sim_no_outline/observations.json",
        )
        st_dict, sph_dict, diam_dict, gyroY = self.load_json(path)
        self.sim_transform(st_dict, sph_dict, gyroY)

    # def test_traj_case1c(self):
    #     path = os.path.join(
    #         FLIGHT_SOFTWARE_PATH,
    #         "OpticalNavigation/simulations/sim/data/traj-case1c_sim_no_outline/observations.json",
    #     )
    #     fileInfo, centerSt, dt, truthT0Vec, _, gyroY = self.get_data(path)
    #     logging.debug(f"{centerSt=}")
    #     calcVecs = self.transform(fileInfo, centerSt, dt, gyroY)
    #     for c in calcVecs:
    #         logging.debug(f"{str(c)=}")


#
#     # truthVecs = [DetectionData(fileInfo, truthT0Vec, ]
#     # for i in fileInfo:
#         # logging.debug(i)


# def test_traj_case1c(self):
# path = os.path.join(
# FLIGHT_SOFTWARE_PATH,
# "OpticalNavigation/simulations/sim/data/traj-case1c_sim_no_outline/observations.json",
# )
# fileInfo, centerSt, dt, truthT0Vec, _, gyroY = self.get_data(path)
# _, _, _ = self.transform(fileInfo, centerSt, dt, truthT0Vec, gyroY)
# for i in fileInfo:
# logging.debug(i)

# def test_traj_trajectory(self):
# path = os.path.join(
# FLIGHT_SOFTWARE_PATH,
# "OpticalNavigation/simulations/sim/data/trajectory_sim/observations.json",
# )
# fileInfo, centerSt, radiusSt, dt, truthT0Vec, truthT0Size, gyroY = self.get_data(path)
# _, _, _ = self.body_meas(fileInfo, centerSt, radiusSt, dt, truthT0Vec, truthT0Size, gyroY)


if __name__ == "__main__":
    unittest.main()
