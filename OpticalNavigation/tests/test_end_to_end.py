import unittest
import os
import glob
import logging
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
        repr = test_reprojections.TestReprojections()

        gn_list, st_list = repr.get_images(path)
        re_path = os.path.join(path, "out/*_re.png")

        re_list = sorted(glob.glob(re_path))
        if len(re_list) < len(gn_list):
            logging.debug("Not enough reprojected images. Running test again.")
            repr.reproj_test(True, False, path)
            re_list = sorted(glob.glob(re_path))

        return gn_list, st_list, re_list

    def find_center_diam(self, img_lst):
        center_find = test_center_finding.CenterDetections()
        cd_dict_st = center_find.calc_centers_and_diam(img_lst)
        return cd_dict_st

    def run_body_meas_sim(self, path, centersReproj):
        """Takes in a path and reprojected centers (from center_finding) and outputs the transformed vectors, as well
        as the truth vectors and the corresponding error"""
        bodyTest = test_body_meas.BodyMeas()
        # Get truth data
        _, truth_dict, diam_dict, gyroY = bodyTest.load_json(path)
        bodyTest.sim_transform(centersReproj, truth_dict, gyroY)
        return diam_dict

        # fileInfo, _, dt, truthT0Vec, truthT0Size, gyroY = bodyTest.get_data(path)
        # logging.debug("fileInfo")
        # for f in fileInfo:
        # logging.debug(str(f))
        # Perform transformations
        # calcVecs, truthVecs, errors = bodyTest.transform(
        # fileInfo, centersReproj, dt, truthT0Vec, gyroY
        # )
        # return calcVecs, truthVecs, errors, fileInfo, truthT0Size

    def test_end_to_end(self):

        # We are testing three things here
        # 1. Taking a gn sim image through reproj->center->body
        # 2. Taking a st sim image through center->body
        # 3. Angular size
        # Then we are comparing the result with the thruth values in the sim logs

        ##############################################################################################################
        # First step: Reprojection
        # Run reprojection test on gnomonic image, output stereographic image from reprojection as well as from sim
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
        logging.debug("Running find on sim stereo images\n")
        cd_dict_st = self.find_center_diam(st_list)
        logging.debug(f"Truth:\n\ncd_dict_st: {cd_dict_st}\n")

        # Get the centers and diamters of reprojected stereo images
        logging.debug("Running find on reprojected images\n")
        cd_dict_re = self.find_center_diam(re_list)
        logging.debug(f"Reproj\ncd_dict_re: {cd_dict_re}\n")
        logging.debug("\n")

        ###############################################################################################################
        # Third step: Stereographic Coordinate Comparison
        # Compare stereographic coordiantes from reprojected image to sim stereo result
        # Both values are generated from the find algorithm
        ###############################################################################################################

        # TODO: Maybe change this to compare to sim json?
        # ... but an equivalent comparison is done in comparing transformed vectors, so might be fine
        logging.debug("Stereographic coordinate comparison")
        for re_key in cd_dict_re.keys():  # Iterate through all detections
            # logging.debug(re_key)
            st_key = re.sub("_re", "_st", re_key)
            # logging.debug(st_key)

            re_dets = cd_dict_re[re_key]
            st_dets = cd_dict_st[st_key]
            # logging.debug(f"Reprojected Detections: {re_dets}")
            # logging.debug(f"Stereogrpahic Detections: {st_dets}")

            for body in (BodyEnum.Earth, BodyEnum.Moon, BodyEnum.Sun):
                if body in re_dets.keys():
                    # assert(BodyEnum.Earth in st_dets.keys(),
                    #   "Reproj detected a body that sim detection did not detect!")
                    if body in st_dets.keys():
                        re_point = np.array((re_dets[body][0], re_dets[body][1]))
                        st_point = np.array((st_dets[body][0], st_dets[body][1]))
                        error = np.linalg.norm(re_point - st_point)
                        logging.debug(
                            f"Reproj File: {re_key} Body: {body} Error: {error}"
                        )
                    else:
                        logging.debug(
                            "Reproj detected a body that sim detection did not detect!"
                        )
        logging.debug("\n")

        ###############################################################################################################
        # Fourth step: Body Measurements
        # Run body_meas test on the two image centers to output body detection vectors
        ###############################################################################################################

        # Runs body_meas_test
        diam_dict = self.run_body_meas_sim(obs_path, cd_dict_re)

        logging.debug("Angular Size comparison")
        for re_key in cd_dict_re.keys():
            logging.debug(f"File: {re_key}")
            for body_key in cd_dict_re[re_key].keys():
                logging.debug(f"Body: {BodyEnum(body_key).name}")
                calc_ang_size = cd_dict_re[re_key][body_key][2]
                logging.debug(f"Calculated Angular size: {calc_ang_size}")
                truth_ang_size = diam_dict[body_key]
                logging.debug(f"Truth Angular Size: {truth_ang_size}")
                diff = abs(calc_ang_size - truth_ang_size)
                logging.debug("Angular Size Diff: %2.6f radians" % diff)

                # percent_error = (abs(calc_ang_size - truth_ang_size) / truth_ang_size) * 100
                # logging.debug(f"Percent Error: {percent_error}")
            logging.debug("")

        # Fourth step: compare difference/error between the actual and test results. How does the error build in each
        # of the three test?
        # Outputs of interest: error in pixel centers from sim and reproj images, error in detection vectors from sim
        # and reproj

        # TODO


if __name__ == "__main__":
    unittest.main()
