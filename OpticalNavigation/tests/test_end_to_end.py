import cv2

# import os
import unittest
import logging
from OpticalNavigation.tests import test_reprojections
from OpticalNavigation.tests import test_body_meas


# from utils.constants import FLIGHT_SOFTWARE_PATH


class TestEndToEnd(unittest.TestCase):
    def get_images_and_reproject(self):

        gn_list, st_list = test_reprojections.get_images()
        gn_imgs, st_imgs, re_imgs = [], [], []

        for idx, gnName in enumerate(gn_list):
            src = cv2.imread(gn_list[idx])
            tgt = cv2.imread(st_list[idx])
            re_img, _ = test_reprojections.reproj(src, gnName)
            gn_imgs.append(src)
            st_imgs.append(tgt)
            re_imgs.append(re_img)

        return gn_imgs, st_imgs, re_imgs

    def run_body_meas_sim(self, path, centersReproj):
        """Takes in a path and reprojected centers (from center_finding) and outputs the transformed vectors, as well
        as the truth vectors and the corresponding error"""
        bodyTest = test_body_meas.BodyMeas()
        fileInfo, _, dt, truthT0Vec, gyroY = bodyTest.get_data(path)
        calcVecs, refVecs, errors = bodyTest.body_meas(
            fileInfo, centersReproj, dt, truthT0Vec, gyroY
        )
        return calcVecs, refVecs, errors

    def test_end_to_end(self, path):

        # TODO I think we should organize is similar to my test_body_meas, where our "test_" functions call this
        # function, except with a different path. This would mean you would have to make your get_images function take
        # in a path a path as an input, but it will allow for us to be more flexible in our testing

        # We are testing three things here
        # 1. Taking a gn sim image through reproj->center->body
        # 2. Taking a st sim image through center->body
        # 3. Angular size
        # Then we are comparing the result with the thruth values in the sim logs

        # First step: run reprojection test on gnomonic image, output stereographic image from reproection as well as
        #             from sim

        gn_imgs, st_imgs, re_imgs = self.get_images_and_reproject()
        logging.debug(gn_imgs)

        # Second step: run center finding test on stereographic image from reprojection test, as well as on
        #              stereographic sim image

        # Third step: run body_meas test on the two image centers to output body detection vectors
        reproj_calc_vecs, reproj_ref_vecs, reproj_errors = self.run_body_meas(path)

        # Fourth step: compare difference/error between the actual and test results. How does the error build in each
        #              of the three test?
        #              Outputs of interest: error in pixel centers from sim and reproj images, error in detection
        #              vectors from sim and reproj

        pass


if __name__ == "__main__":
    unittest.main()
