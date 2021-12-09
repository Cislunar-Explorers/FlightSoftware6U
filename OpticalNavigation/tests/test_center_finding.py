import json
import csv
import os
from OpticalNavigation.core.find_algos.find_with_hough_transform_and_contours import (
    find,
)
from matplotlib import pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
import math
import unittest

# import argparse


class CenterDetections(unittest.TestCase):
    def __get_difference(self, body, truths, body_vals):
        """
        Gets the absolute differences between the truth values and the returned body values. Returns perf (performance)
        values as a dictionary, one for each body detected and compared.
        """
        perf_values = {}
        truth_vals = truths[body]
        perf = []
        distance = (
            (body_vals[0] - truth_vals[0]) ** 2 + (body_vals[1] - truth_vals[1]) ** 2
        ) ** 0.5
        perf += [distance, abs(body_vals[2] - truth_vals[2])]
        perf_values[body] = perf
        return perf_values

    # TODO: Allow a different find algorithm to be tested easily
    def get_results(self, dir, results_file, st_gn, pixel=True):
        """
        The main function for feature testing center finding. The function takes in a directory where all the files
        are located (images, observations.json, and cameras.json files). It then uses the find algorithm to determine
        the centers on all the camera frames in the images directory and compares these values with the truth values
        in observations.json. cameras.json is required in order to get the correct values for the stereographic
        scaling. It then writes these values in a csv format to a given csv file.
        """
        # Getting frames and json files
        frames = []
        frames_dir = os.path.join(dir, "images/")
        for filename in os.listdir(frames_dir):
            if filename.endswith(st_gn + ".png"):
                frame = os.path.join(frames_dir, filename)
                frames.append(frame)
            else:
                continue
        o = open(os.path.join(dir, "observations.json"))
        c = open(os.path.join(dir, "cameras.json"))
        observations = json.load(o)
        cameras = json.load(c)
        if pixel:
            st_scale = cameras["cameras"][0]["stereographic_scale"]
        else:
            st_scale = 1

        # Reading in truth values from observations.json
        all_truth_vals = {}
        for i in range(len(observations["observations"])):
            for frame in observations["observations"][i]["frames"]:
                detections = frame["detections"]
                frame_truth_vals = {}
                for detection in detections:
                    truthX = detection["center_st"][0] * st_scale
                    truthY = detection["center_st"][1] * st_scale
                    truthR = detection["radius_st"] * st_scale
                    frame_truth_vals[detection["body"]] = [truthX, truthY, truthR]
                image_type = (
                    "image_stereographic" if st_gn == "st" else "image_gnomonic"
                )
                all_truth_vals[frame[image_type]] = frame_truth_vals
        results = []
        # Comparing found values with truth values
        for i in range(len(frames)):
            frame = frames[i]
            truths = all_truth_vals[frame.split("/")[-1]]
            _, body_vals = (
                find(frame, st=True, pixel=pixel)
                if st_gn == "st"
                else find(frame, pixel=pixel)
            )
            sun_vals = body_vals.get("Sun")
            if sun_vals:
                perf_values = self.__get_difference("Sun", truths, sun_vals)
                results.append(perf_values)
            earth_vals = body_vals.get("Earth")
            if earth_vals:
                perf_values = self.__get_difference("Earth", truths, earth_vals)
                results.append(perf_values)
            moon_vals = body_vals.get("Moon")
            if moon_vals:
                perf_values = self.__get_difference("Moon", truths, moon_vals)
                results.append(perf_values)

        # Writing performance values to csv file
        with open(results_file, "w", newline="") as csvfile:
            writer = csv.writer(
                csvfile, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL
            )
            writer.writerow(["Body", "Center", "Radius"])
            for result in results:
                sun = result.get("Sun")
                earth = result.get("Earth")
                moon = result.get("Moon")
                if sun is not None:
                    writer.writerow(["Sun"] + sun)
                if earth is not None:
                    writer.writerow(["Earth"] + earth)
                if moon is not None:
                    writer.writerow(["Moon"] + moon)
        return results

    def __diff_histogram(self, results, center, filename, show, name, st_gn):
        """
        Generates the histogram of the difference between the truth value and the found value.
        """
        idx = 0 if center else 1
        center_radius = "Center" if center else "Radius"
        title = "{} Absolute Difference {} {}".format(center_radius, name, st_gn)
        data = []
        for result in results:
            sun = result.get("Sun")
            earth = result.get("Earth")
            moon = result.get("Moon")
            if sun is not None:
                data.append(sun[idx])
            if earth is not None:
                data.append(earth[idx])
            if moon is not None:
                data.append(moon[idx])
        fig = plt.figure()
        ax = fig.add_subplot()
        ax.hist(data, bins=20, range=(0, 5), edgecolor="black", linewidth=1.2)
        ax.set_xlim(left=0)
        ax.xaxis.set_major_locator(MaxNLocator(integer=False))
        ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        x_min, x_max = ax.get_xlim()
        plt.xticks(np.arange(x_min, x_max + 1, 0.5))
        ax.title.set_text(title)
        plt.xlabel("Pixel Distance (px)")
        plt.ylabel("Number of Results")
        fig.savefig(filename)
        if show:
            plt.show()
        return data

    def center_histogram(self, results, filename, name, show=False, st_gn="st"):
        """
        Generates the histogram for center results.
        """
        return self.__diff_histogram(results, True, filename, show, name, st_gn)

    def radius_histogram(self, results, filename, name, show=False, st_gn="st"):
        """
        Generates the histogram for radius results.
        """
        return self.__diff_histogram(results, False, filename, show, name, st_gn)

    def center_finding_results(
        self,
        dir,
        results_file,
        center_histogram_file,
        radius_histogram_file,
        st_gn="st",
    ):
        name = dir.split("/")[-1]
        results = self.get_results(dir, results_file, st_gn)
        center_data = self.center_histogram(
            results, center_histogram_file, name, st_gn=st_gn
        )
        radius_data = self.radius_histogram(
            results, radius_histogram_file, name, st_gn=st_gn
        )
        total_detections = len(center_data)
        correct_detections = 0
        # TODO: make the thresholds below into parameters
        for i in range(len(center_data)):
            if center_data[i] <= math.sqrt(8) and radius_data[i] <= 2:
                correct_detections += 1
        passing_detections = correct_detections / total_detections
        self.assertGreaterEqual(
            passing_detections,
            0.70,
            "Center find algorithm is not at least 70% accurate!",
        )

    def test_traj_case_1c(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../simulations/sim/data/traj-case1c_sim")
        return self.center_finding_results(
            filename,
            "center_finding_results_traj_case_1c_sim.csv",
            "center_histogram_traj_case1c.png",
            "radius_histogram_traj_case1c.png",
        )

    def test_trajectory(self):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, "../simulations/sim/data/trajectory_sim")
        return self.center_finding_results(
            filename,
            "center_finding_results_trajectory_sim.csv",
            "center_histogram_trajectory.png",
            "radius_histogram_trajectory.png",
        )


if __name__ == "__main__":
    unittest.main()

# if __name__ == "__main__":
#     """
#     Run "python3 test_center_finding.py -d=<DIRECTORY> -r=<RESULTS_FILE>
#     -ch=<CENTER_HISTOGRAM_FILE> -rh=<RADIUS_HISTOGRAM_FILE> [-st_gn=<STEREOGRAPHIC_OR_GNOMONIC>]" to test this module
#     """
#     ap = argparse.ArgumentParser()
#     ap.add_argument("-d", "--directory", help="path to trajectory directory")
#     ap.add_argument("-r", "--results_file", help="results file to output results")
#     ap.add_argument(
#         "-ch",
#         "--center_histogram_file",
#         help="histogram file to output center results histogram",
#     )
#     ap.add_argument(
#         "-rh",
#         "--radius_histogram_file",
#         help="histogram file to output radius results histogram",
#     )
#     ap.add_argument(
#         "-st_gn",
#         "--stereographic_or_gnomonic",
#         nargs="?",
#         help="whether this is stereographic or gnomonic",
#     )
#     args = vars(ap.parse_args())
#     st_gn = args.get("st_gn")
#     if st_gn:
#         center_finding_results(
#             args["directory"],
#             args["results_file"],
#             args["center_histogram_file"],
#             args["radius_histogram_file"],
#             st_gn,
#         )
#     else:
#         center_finding_results(
#             args["directory"],
#             args["results_file"],
#             args["center_histogram_file"],
#             args["radius_histogram_file"],
#         )