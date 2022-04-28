import unittest
import os
import glob
import logging
from OpticalNavigation.tests import (
    test_center_finding,
    test_reprojections,
    test_body_meas,
)
from utils.constants import FLIGHT_SOFTWARE_PATH

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
        fileInfo, _, dt, truthT0Vec, truthT0Size, gyroY = bodyTest.get_data(path)
        # logging.debug("fileInfo")
        # for f in fileInfo:
        # logging.debug(str(f))
        # Perform transformations
        calcVecs, truthVecs, errors = bodyTest.body_meas(
            fileInfo, centersReproj, dt, truthT0Vec, gyroY
        )
        return calcVecs, truthVecs, errors, fileInfo, truthT0Size

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

        centers_st = []
        ang_diams_st = []
        for i in cd_dict_st.keys():
            key = list(cd_dict_st[i].keys())[0]
            # logging.debug(cd_dict_st[i])
            # logging.debug(key)
            centers_st.append([cd_dict_st[i][key][0], cd_dict_st[i][key][1]])
            ang_diams_st.append(cd_dict_st[i][key][2])
        logging.debug(f"Truth:\n\ncd_dict_st: {cd_dict_st}\n")
        # logging.debug(f"centers_st: {centers_st}\n")
        # logging.debug(f"ang_diams_st: {ang_diams_st}\n")

        # Get the centers and diamters of reprojected stereo images
        logging.debug("Running find on reprojected images\n")
        cd_dict_re = self.find_center_diam(re_list)

        # logging.debug(cd_dict_re)
        centers_re = []
        ang_diams_re = []
        for i in cd_dict_re.keys():
            # Check if cr_dict_re[i].keys() isn't empty; i.e., no detections
            if len(cd_dict_re[i].keys()) != 0:
                key = list(cd_dict_re[i].keys())[0]
                # logging.debug(cd_dict_re[i])
                # logging.debug(key)
                centers_re.append([cd_dict_re[i][key][0], cd_dict_re[i][key][1]])
                ang_diams_re.append(cd_dict_re[i][key][2])
        logging.debug(f"Reproj\ncd_dict_re: {cd_dict_re}\n")
        # logging.debug(f"centers_re: {centers_re}\n")
        # logging.debug(f"ang_diams_re: {ang_diams_re}\n")
        logging.debug("\n")

        ###############################################################################################################
        # Third step: Body Measurements
        # Run body_meas test on the two image centers to output body detection vectors
        ###############################################################################################################

        re_calc_vecs, re_truth_vecs, re_errors, fileInfo, truth_sizes = self.run_body_meas_sim(
            obs_path, centers_re
        )

        # logging.debug("Size comparison")
        # for f, a, t in zip(fileInfo, ang_diams_re, truth_sizes):
        # logging.debug(f"({str(f)}, Meas: {a}, Actual: {t})")
        # percent_error = (abs(a - t) / t) * 100
        # logging.debug(f"Percent Error: {percent_error}")

        ###############################################################################################################
        # Fourth step: Comparisons
        # Compare reproj centers and sim stereo results
        ###############################################################################################################

        # for

        # Fourth step: compare difference/error between the actual and test results. How does the error build in each
        # of the three test?
        # Outputs of interest: error in pixel centers from sim and reproj images, error in detection vectors from sim
        # and reproj

        # TODO


if __name__ == "__main__":
    unittest.main()
