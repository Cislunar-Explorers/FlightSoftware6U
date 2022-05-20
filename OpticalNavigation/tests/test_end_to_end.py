import unittest
import os
import glob

# import logging
from utils.log import log
from OpticalNavigation.tests import (
    test_center_finding,
    test_reprojections,
    test_body_meas,
)
from core.const import BodyEnum
from utils.constants import FLIGHT_SOFTWARE_PATH
import re
import numpy as np

DATA_DIR = str(FLIGHT_SOFTWARE_PATH) + "/OpticalNavigation/simulations/sim/data"


class TestEndToEnd(unittest.TestCase):
    def reproject_images(self, path):
        """
        Performs reprojection on gnomonic images
        """
        repr = test_reprojections.TestReprojections()

        gn_list, st_list = repr.get_images(path)
        re_path = os.path.join(path, "out/*_re.png")

        re_list = sorted(glob.glob(re_path))
        if len(re_list) < len(gn_list):
            log.debug("Not enough reprojected images. Running test again.")
            repr.reproj_test(True, False, path)
            re_list = sorted(glob.glob(re_path))

        return gn_list, st_list, re_list

    def find_center_diam(self, img_lst):
        """
        Runs find algorithm and returns detected center and angular diamter information
        """
        center_find = test_center_finding.CenterDetections()
        cd_dict_st = center_find.calc_centers_and_diam(img_lst)
        return cd_dict_st

    def run_body_meas_sim(self, path, centersReproj):
        """
        Takes in a path and reprojected centers (from center_finding) and outputs the transformed vectors, as well
        as the truth vectors and the corresponding error
        Returns truth diam information to be used for testing done later (we read the json in this function)
        """
        bodyTest = test_body_meas.BodyMeas()
        # Get truth data
        _, truth_dict, diam_dict, gyroY = bodyTest.load_json(path)
        bodyTest.transform(centersReproj, truth_dict, gyroY)
        return diam_dict

    def test_end_to_end(self):
        ##############################################################################################################
        # First step: Reprojection
        # Run reprojection test on gnomonic image, outputs stereographic image from reprojection
        # as well stereographic image from sim datasets
        ##############################################################################################################

        imgs_path = os.path.join(DATA_DIR, "traj-case1c_sim_no_outline")
        gn_list, st_list, re_list = self.reproject_images(imgs_path)

        obs_path = os.path.join(
            DATA_DIR, "traj-case1c_sim_no_outline/observations.json"
        )

        ###############################################################################################################
        # Second step: Center Finding
        # Run center finding test on sim stereographic images, as well as on reprojected stereographic images
        ##############################################################################################################

        # Get the centers and diameters of truth sim stereo images
        log.debug("Running find on sim stereo images\n")
        cd_dict_st = self.find_center_diam(st_list)
        log.debug(f"Truth:\ncd_dict_st: {cd_dict_st}\n")

        # Get the centers and diamters of reprojected stereo images
        log.debug("Running find on reprojected images\n")
        cd_dict_re = self.find_center_diam(re_list)
        log.debug(f"Reproj:\ncd_dict_re: {cd_dict_re}\n")
        log.debug("\n")

        ###############################################################################################################
        # Third step: Stereographic Coordinate Comparison
        # Compare stereographic coordiantes from reprojected image to sim stereo result
        # Both values are generated from the find algorithm
        ###############################################################################################################

        # TODO: Maybe change this to compare to sim json?
        # ... but an equivalent comparison is done in comparing transformed vectors, so might be fine
        log.debug("Stereographic coordinate comparison")
        for re_key in cd_dict_re.keys():  # Iterate through all detections
            st_key = re.sub("_re", "_st", re_key)

            re_dets = cd_dict_re[re_key]
            st_dets = cd_dict_st[st_key]

            for body in (BodyEnum.Earth, BodyEnum.Moon, BodyEnum.Sun):
                if body in re_dets.keys():
                    # assert(BodyEnum.Earth in st_dets.keys(),
                    #   "Reproj detected a body that sim detection did not detect!")
                    if body in st_dets.keys():
                        re_point = np.array((re_dets[body][0], re_dets[body][1]))
                        st_point = np.array((st_dets[body][0], st_dets[body][1]))
                        error = np.linalg.norm(re_point - st_point)
                        log.debug(f"Reproj File: {re_key} Body: {body} Error: {error}")
                    else:
                        log.debug(
                            "Reproj detected a body that sim detection did not detect!"
                        )
        log.debug("\n")

        ###############################################################################################################
        # Fourth step: Body Measurements
        # Run body_meas test on the two image centers to output body detection vectors
        # Logging is done within test_body_meas.py
        ###############################################################################################################

        # Runs body_meas_test, also returns diameter data for later tests
        diam_dict = self.run_body_meas_sim(obs_path, cd_dict_re)

        # Compare angular size of detected bodies
        log.debug("Angular Size comparison")
        for re_key in cd_dict_re.keys():
            log.debug(f"File: {re_key}")
            for body_key in cd_dict_re[re_key].keys():
                log.debug(f"Body: {BodyEnum(body_key).name}")
                calc_ang_size = cd_dict_re[re_key][body_key][2]
                log.debug(f"Calculated Angular size: {calc_ang_size}")
                truth_ang_size = diam_dict[body_key]
                log.debug(f"Truth Angular Size: {truth_ang_size}")
                diff = abs(calc_ang_size - truth_ang_size)
                log.debug(
                    "Angular Size Diff: %2.8f rad, %2.8f deg" % (diff, np.rad2deg(diff))
                )
                log.debug(
                    "Percent Error: %2.8f"
                    % (100 * (calc_ang_size - truth_ang_size) / truth_ang_size)
                )

                # percent_error = (abs(calc_ang_size - truth_ang_size) / truth_ang_size) * 100
                # logging.debug(f"Percent Error: {percent_error}")
            log.debug("")


if __name__ == "__main__":
    unittest.main()
