import cv2

# import os
import unittest
import logging
import os
from OpticalNavigation.tests import (
    test_center_finding,
    test_reprojections,
    test_body_meas,
)
from utils.constants import FLIGHT_SOFTWARE_PATH

DATA_DIR = str(FLIGHT_SOFTWARE_PATH) + "/OpticalNavigation/simulations/sim/data/"


# from utils.constants import FLIGHT_SOFTWARE_PATH


class TestEndToEnd(unittest.TestCase):
    def get_images_and_reproject(self, path):

        repr = test_reprojections.TestReprojections()

        gn_list, st_list = repr.get_images(path)
        gn_imgs, st_imgs, re_imgs = {}, {}, {}

        for idx, gnName in enumerate(gn_list):
            print(gnName)
            src = cv2.imread(gn_list[idx])
            tgt = cv2.imread(st_list[idx])
            re_img, _ = repr.reproj(src, gnName)
            gn_imgs[gnName] = src
            st_imgs[gnName.replace("_gn.png", "_st.png")] = tgt
            re_imgs[gnName.replace("_gn.png", "_re.png")] = re_img

        return gn_imgs, st_imgs, re_imgs

    def get_center_radius(self, img_lst):
        center_find = test_center_finding.CenterDetections()

        cr_dict = center_find.calc_centers_and_radii(img_lst)

        return cr_dict

    def run_body_meas_sim(self, path, centersReproj):
        """Takes in a path and reprojected centers (from center_finding) and outputs the transformed vectors, as well
        as the truth vectors and the corresponding error"""
        bodyTest = test_body_meas.BodyMeas()
        fileInfo, _, dt, truthT0Vec, gyroY = bodyTest.get_data(path)
        calcVecs, refVecs, errors = bodyTest.body_meas(
            fileInfo, centersReproj, dt, truthT0Vec, gyroY
        )
        return calcVecs, refVecs, errors

    def test_end_to_end(self):

        # We are testing three things here
        # 1. Taking a gn sim image through reproj->center->body
        # 2. Taking a st sim image through center->body
        # 3. Angular size
        # Then we are comparing the result with the thruth values in the sim logs

        # First step: run reprojection test on gnomonic image, output stereographic image from reprojection as well as
        #             from sim

        path = os.path.join(DATA_DIR, "traj-case1c_sim_no_outline/images/*_gn.png")

        gn_imgs, st_imgs, re_imgs = self.get_images_and_reproject(path)
        logging.debug(re_imgs)

        # Second step: run center finding test on stereographic image from reprojection test, as well as on
        #              stereographic sim image
        # cr_dict = self.get_center_radius(re_imgs)

        # Third step: run body_meas test on the two image centers to output body detection vectors
        reproj_calc_vecs, reproj_ref_vecs, reproj_errors = self.run_body_meas_sim(path)

        # Fourth step: compare difference/error between the actual and test results. How does the error build in each
        #              of the three test?
        #              Outputs of interest: error in pixel centers from sim and reproj images, error in detection
        #              vectors from sim and reproj

        pass


if __name__ == "__main__":
    unittest.main()
