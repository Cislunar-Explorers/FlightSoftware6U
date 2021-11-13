import json
import csv
import os
from math import dist
from OpticalNavigation.core.find_algos.find_with_hough_transform_and_contours import (
    findStereographic,
)
from matplotlib import pyplot as plt


def get_difference(body, truths, body_vals):
    perf_values = {}
    truth_vals = truths[body]
    perf = []
    distance = dist((body_vals[0], body_vals[1]), (truth_vals[0], truth_vals[1]))
    perf += [distance, abs(body_vals[2] - truth_vals[2])]
    perf_values[body] = perf
    return perf_values


def test_center_finding(dir, results_file, st_gn="st"):
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
    st_scale = cameras["cameras"][0]["stereographic_scale"]
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
            all_truth_vals[frame["image_stereographic"]] = frame_truth_vals
    results = []
    for i in range(len(frames)):
        frame = frames[i]
        truths = all_truth_vals[frame.split("/")[-1]]
        _, body_vals = findStereographic(frame)
        sun_vals = body_vals.get("Sun")
        if sun_vals:
            perf_values = get_difference("Sun", truths, sun_vals)
            results.append(perf_values)
        earth_vals = body_vals.get("Earth")
        if earth_vals:
            perf_values = get_difference("Earth", truths, earth_vals)
            results.append(perf_values)
        moon_vals = body_vals.get("Moon")
        if moon_vals:
            perf_values = get_difference("Moon", truths, moon_vals)
            results.append(perf_values)
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


def __diff_histogram(results, center, filename, show):
    idx = 1 if center else 0
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
    plt.hist(data, bins=20, range=(0, 5), edgecolor="black", linewidth=1.2)
    fig.savefig(filename)
    if show:
        plt.show()
    return data


def center_histogram(results, filename, show=False):
    return __diff_histogram(results, True, filename, show)


def radius_histogram(results, filename, show=False):
    return __diff_histogram(results, False, filename, show)


# traj_case1c_sim_easy = "/Users/andrew/PycharmProjects/opnav_sim/traj-case1c_sim_easy/"
# traj_case1c_sim_easy_results_file = "center_finding_results_traj-case1c_sim_easy.csv"
# test_center_finding(traj_case1c_sim_easy, traj_case1c_sim_easy_results_file)

traj_case1c_sim = "/Users/andrew/PycharmProjects/opnav_sim/traj-case1c_sim/"
traj_case1c_sim_results_file = "center_finding_results_traj-case1c_sim.csv"
results_traj_case1c = test_center_finding(traj_case1c_sim, traj_case1c_sim_results_file)
center_histogram(results_traj_case1c, "center_histogram_traj_case1c.png")
radius_histogram(results_traj_case1c, "radius_histogram_traj_case1c.png")

# trajectory_sim_easy = "/Users/andrew/PycharmProjects/opnav_sim/trajectory_sim_easy/"
# trajectory_sim_easy_results_file = "center_finding_results_trajectory_sim_easy.csv"
# test_center_finding(trajectory_sim_easy, trajectory_sim_easy_results_file)

trajectory_sim = "/Users/andrew/PycharmProjects/opnav_sim/trajectory_sim/"
trajectory_sim_results_file = "center_finding_results_trajectory_sim.csv"
results_trajectory = test_center_finding(trajectory_sim, trajectory_sim_results_file)
center_histogram(results_trajectory, "center_histogram_trajectory.png")
radius_histogram(results_trajectory, "radius_histogram_trajectory.png")
