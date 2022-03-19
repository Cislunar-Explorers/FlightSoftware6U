import unittest
import os
import glob
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

        gn_list, st_list = repr.get_images(os.path.join(path, "images"))
        re_path = os.path.join(path, "out/*_re.png")

        re_list = sorted(glob.glob(re_path))
        if len(re_list) < len(gn_list):
            print("Not enough reprojected images. Running test again.")
            repr.reproj_test(True, False, path)
            re_list = sorted(glob.glob(re_path))

        return gn_list, st_list, re_list

    def get_center_radius(self, img_lst):
        center_find = test_center_finding.CenterDetections()

        cr_dict_st = center_find.calc_centers_and_radii(img_lst)

        return cr_dict_st

    def run_body_meas_sim(self, path, centersReproj, radiiSt):
        """Takes in a path and reprojected centers (from center_finding) and outputs the transformed vectors, as well
        as the truth vectors and the corresponding error"""
        bodyTest = test_body_meas.BodyMeas()
        fileInfo, _, _, dt, truthT0Vec, truthT0Size, gyroY = bodyTest.get_data(path)
        calcVecs, refVecs, errors = bodyTest.body_meas(
            fileInfo, centersReproj, radiiSt, dt, truthT0Vec, truthT0Size, gyroY
        )
        return calcVecs, refVecs, errors

    def test_end_to_end(self):

        # We are testing three things here
        # 1. Taking a gn sim image through reproj->center->body
        # 2. Taking a st sim image through center->body
        # 3. Angular size
        # Then we are comparing the result with the thruth values in the sim logs

        # First step: run reprojection test on gnomonic image, output stereographic image from reprojection as well as
        # from sim.

        imgs_path = os.path.join(DATA_DIR, "traj-case1c_sim_no_outline")
        gn_list, st_list, re_list = self.reproject_images(imgs_path)

        obs_path = os.path.join(
            DATA_DIR, "traj-case1c_sim_no_outline/observations.json"
        )

        # Second step: run center finding test on sim stereographic images, as well as on
        # reprojected stereographic images.

        # Get the centers and radii of sim stereo images
        cr_dict_st = self.get_center_radius(st_list)
        # print(cr_dict_st)
        centers_st = []
        radii_st = []
        for i in cr_dict_st.keys():
            key = list(cr_dict_st[i].keys())[0]
            # print(cr_dict_st[i])
            # print(key)
            centers_st.append([cr_dict_st[i][key][0], cr_dict_st[i][key][1]])
            radii_st.append(cr_dict_st[i][key][2])
        print(f"centers_st:\n{centers_st}")
        print(f"radii_st\n{radii_st}")

        # Get the centers and radii of reprojected stereo images
        cr_dict_re = self.get_center_radius(re_list)
        # print(cr_dict_re)
        centers_re = []
        radii_re = []
        for i in cr_dict_re.keys():
            # Check if cr_dict_re[i].keys() isn't empty; i.e., no detections
            if len(cr_dict_re[i].keys()) != 0:
                key = list(cr_dict_re[i].keys())[0]
                # print(cr_dict_re[i])
                # print(key)
                centers_re.append([cr_dict_re[i][key][0], cr_dict_re[i][key][1]])
                radii_re.append(cr_dict_re[i][key][2])
        print(f"centers_re:\n{centers_re}")
        print(f"radii_re\n{radii_re}")

        # Third step: run body_meas test on the two image centers to output body detection vectors
        reproj_calc_vecs, reproj_ref_vecs, reproj_errors = self.run_body_meas_sim(
            obs_path, centers_st, radii_st
        )

        st_calc_vecs, st_ref_vecs, st_errors = self.run_body_meas_sim(
            obs_path, centers_re, radii_re
        )

        # Fourth step: compare difference/error between the actual and test results. How does the error build in each
        # of the three test?
        # Outputs of interest: error in pixel centers from sim and reproj images, error in detection vectors from sim
        # and reproj

        print("TODO")


if __name__ == "__main__":
    unittest.main()
