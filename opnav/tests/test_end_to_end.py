import unittest
import os
import glob

# import logging
from utils.log import log
from opnav.tests import (
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

    def stereo_compare(self, cd_dict_re, cd_dict_st):
        """
        This function performs a comparison between the stereographic coordinates output by the find algorithm
        In step two of test_end_to_end, the find algorithm was run on both the reporjected image, as well as the
        stereographic image generated by the sim. Here the results are compared. This evaluates the perormance of the
        find algorithm on an ideal stereographic image (sim) and a realistic stereographic image (reprojected w/ FSW)
        cd_dict_re holds the reprojected image data, while cd_dict_st holds the stereographic image data
        """
        log.debug("\n")
        log.debug("Stereographic Coordinate Comparison")
        for re_key in cd_dict_re.keys():  # Iterate through all files
            st_key = re.sub("_re", "_st", re_key)  # Replace "_re" with "_st" to get stereographic filename

            re_dets = cd_dict_re[re_key]
            st_dets = cd_dict_st[st_key]

            for body in (
                BodyEnum.Earth,
                BodyEnum.Moon,
                BodyEnum.Sun,
            ):  # Iterate through each possible detection
                if body in re_dets.keys():
                    if body in st_dets.keys():
                        # If body is in both images, calculate error between centers
                        re_point = np.array((re_dets[body][0], re_dets[body][1]))
                        st_point = np.array((st_dets[body][0], st_dets[body][1]))
                        error = np.linalg.norm(re_point - st_point)
                        self.assertLessEqual(
                            error,
                            0.040,
                            (
                                "Stereographic centers do not match within margin of error between running find on "
                                "reprojected and sim stereo images!"
                            ),
                        )
                        log.debug(f"Reproj File: {re_key} Body: {body} Error: {error}")
                    else:
                        log.debug("Reproj detected a body that sim detection did not detect!")
        log.debug("\n")

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

    def angular_size_compare(self, cd_dict_re, diam_dict):
        """
        This function performs a comparison between the angular size calulcated by FSW and the truth values from the
        opnav sim. The diam_dict holds the truth angular sizes for the earth, moon and sun
        """
        for re_key in cd_dict_re.keys():  # Iterate through all filenames
            log.debug(f"File: {re_key}")
            for body_key in cd_dict_re[re_key].keys():  # For each body in the detection
                log.debug(f"Body: {BodyEnum(body_key).name}")

                calc_ang_size = cd_dict_re[re_key][body_key][2]
                log.debug(f"Calculated Angular size: {calc_ang_size}")

                truth_ang_size = diam_dict[body_key]
                log.debug(f"Truth Angular Size: {truth_ang_size}")

                diff = abs(calc_ang_size - truth_ang_size)
                log.debug("Angular Size Diff: %2.8f rad, %2.8f deg" % (diff, np.rad2deg(diff)))
                percent_err = 100 * (calc_ang_size - truth_ang_size) / truth_ang_size
                log.debug("Percent Error: %2.8f%%\n" % percent_err)

                # We have a 20% threshold for moon and 5% threshold for earth and sun based on testing.
                # TODO: In the future we will need to reduce these thresholds to what we can reasonably tolerate
                if BodyEnum(body_key) is BodyEnum.Moon:
                    self.assertLessEqual(
                        abs(percent_err),
                        20,
                        "Angular size result is not within margin of error for moon!",
                    )
                else:
                    self.assertLessEqual(
                        abs(percent_err),
                        5,
                        "Angular size result is not within margin of error for earth or sun!",
                    )

    def test_end_to_end(self):
        ##############################################################################################################
        # First step: Reprojection
        # Run reprojection test on gnomonic image, outputs stereographic image from reprojection
        # as well stereographic image from sim datasets
        ##############################################################################################################
        imgs_path = os.path.join(DATA_DIR, "traj-case1c_sim_no_outline")
        gn_list, st_list, re_list = self.reproject_images(imgs_path)

        obs_path = os.path.join(DATA_DIR, "traj-case1c_sim_no_outline/observations.json")

        ###############################################################################################################
        # Second step: Center Finding
        # Run center finding test on sim stereographic images, as well as on reprojected stereographic images
        ##############################################################################################################
        # Get the centers and diameters of truth sim stereo images
        log.debug("Running find on sim stereo images\n")
        cd_dict_st = self.find_center_diam(st_list)
        # log.debug(f"Truth:\ncd_dict_st: {cd_dict_st}\n")

        # Get the centers and diamters of reprojected stereo images
        log.debug("Running find on reprojected images\n")
        cd_dict_re = self.find_center_diam(re_list)
        # log.debug(f"Reproj:\ncd_dict_re: {cd_dict_re}\n\n")

        ###############################################################################################################
        # Third step: Stereographic Coordinate Comparison
        # Compare stereographic coordiantes from reprojected image to sim stereo result
        # Both values are generated from the find algorithm
        ###############################################################################################################
        # TODO: Maybe change this to compare to sim json?
        # ... but an equivalent comparison is done in comparing transformed vectors, so might be fine
        self.stereo_compare(cd_dict_re, cd_dict_st)

        ###############################################################################################################
        # Fourth step: Body Measurements
        # Run body_meas test on the two image centers to output body detection vectors
        # Logging is done within test_body_meas.py
        # Angular size data is returned for later tests
        ###############################################################################################################
        log.debug("Body Transformation Test")
        diam_dict = self.run_body_meas_sim(obs_path, cd_dict_re)

        ###############################################################################################################
        # Fifth step: Angular Size Comparison
        # Copare calculated angular size to truth values from sim
        ###############################################################################################################
        log.debug("\n")
        log.debug("Angular Size Comparison")
        self.angular_size_compare(cd_dict_re, diam_dict)


if __name__ == "__main__":
    unittest.main()