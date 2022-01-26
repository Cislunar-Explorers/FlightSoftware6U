from core.const import BodyEnum, FileData, DetectionData, Vector3
from core.find_with_contours import *

import numpy as np
import math
import logging


# Make this run on a singular frame, update detections list outside of function, in observe
def get_detections(frames):
    logging.info("[OPNAV]: Finding...")
    progress = 1
    detections = []
    for f in range(len(frames)):
        logging.info(f"[OPNAV]: Image {progress}/{len(frames)}: {frames[f]}")
        fileInfo = FileData(frames[f])
        detectedBodies = find(frames[f])

        # Make this small part into a template function that can be reused for e/m/s
        earthDetection = detectedBodies.get_earth_detection()
        if earthDetection is not None:
            detectionData = DetectionData(
                fileInfo,
                Vector3(earthDetection[0], earthDetection[1], earthDetection[2]),
                earthDetection[3],
                BodyEnum.Earth,
            )
            detections.append(detectionData)
            logging.info(f"[OPNAV]: Earth: {earthDetection}")

        moonDetection = detectedBodies.get_moon_detection()
        if moonDetection is not None:
            detectionData = DetectionData(
                fileInfo,
                Vector3(moonDetection[0], moonDetection[1], moonDetection[2]),
                moonDetection[3],
                BodyEnum.Moon,
            )
            detections.append(detectionData)
            logging.info(f"[OPNAV]: Moon: {moonDetection}")

        sunDetection = detectedBodies.get_sun_detection()
        if sunDetection is not None:
            detectionData = DetectionData(
                fileInfo,
                Vector3(sunDetection[0], sunDetection[1], sunDetection[2]),
                sunDetection[3],
                BodyEnum.Sun,
            )
            detections.append(detectionData)
            logging.info(f"[OPNAV]: Sun: {sunDetection}")

        progress += 1

    return detections


def get_best_detection(detections):
    # (distance to center, DetectionData object)
    closest_e = (np.inf, None)
    closest_m = (np.inf, None)
    closest_s = (np.inf, None)
    for d in detections:
        center_dist = math.sqrt(d.vector.x ** 2 + d.vector.y ** 2)
        if d.detection == BodyEnum.Earth and center_dist < closest_e[0]:
            closest_e = (center_dist, d)
        elif d.detection == BodyEnum.Moon and center_dist < closest_m[0]:
            closest_m = (center_dist, d)
        elif d.detection == BodyEnum.Sun and center_dist < closest_s[0]:
            closest_s = (center_dist, d)
    return closest_e[1], closest_m[1], closest_s[1]


def cam_to_body(detection, camera_params):
    camNum = detection.filedata.cam_num
    if camNum == 1:
        res = (camera_params.cam1Rotation).dot(detection.vector.data)
        detection.vector = Vector3(res[0], res[1], res[2])
    elif camNum == 2:
        res = (camera_params.cam2Rotation).dot(detection.vector.data)
        detection.vector = Vector3(res[0], res[1], res[2])
    elif camNum == 3:
        res = (camera_params.cam3Rotation).dot(detection.vector.data)
        detection.vector = Vector3(res[0], res[1], res[2])
    return detection


def tZeroRotMatrix(rotation):
    """Creates a y-axis rotation matrix"""
    return np.array(
        [
            math.cos(rotation),
            0,
            math.sin(rotation),
            0,
            1,
            0,
            -1 * math.sin(rotation),
            0,
            math.cos(rotation),
        ]
    ).reshape(3, 3)


def get_elapsed_time(fileData, timeDeltaAvgs, observeStart):
    """
    Calculates the elapsed time (in seconds, floating pt) between the beginning of the opnav observe call and the
    timestamp of a selected frame
    """
    timestamp = fileData.timestamp
    camNum = fileData.cam_num
    timestampUnix = timestamp + timeDeltaAvgs[camNum - 1]
    timeElapsed = (timestampUnix - observeStart) * 10 ** -6
    return timeElapsed


def body_to_T0(detection, timeElapsed, avgGyroY):
    logging.info(f"[OPNAV]: {detection.detection} Time Elapsed= {timeElapsed}")
    rotation = avgGyroY * timeElapsed
    tZeroRotation = tZeroRotMatrix(rotation)
    res = (tZeroRotation).dot(detection.vector.data)
    detection.vector = Vector3(res[0], res[1], res[2])
    return detection
