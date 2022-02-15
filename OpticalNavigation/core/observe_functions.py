from core.const import BodyEnum, FileData, DetectionData, Vector3
from core.find_algos.find_with_contours import *
from core.sense import select_camera, record_video

from adafruit_blinka.agnostic import board_id

if board_id and board_id != "GENERIC_LINUX_PC":
    from picamera import (
        PiCamera,
    )  # TODO: only commented because of commented out camera section

    # pass
from utils.constants import OPNAV_MEDIA_DIR

import numpy as np
import math
import time
import logging
from astropy.time import Time
from astropy.coordinates import get_sun, get_moon, CartesianRepresentation


def record_with_timedelta(camNum, camera_rec_params):
    select_camera(id=camNum)

    # Get Unix time before recording(in seconds floating point -> microseconds)
    # Get camera time (in microseconds)
    linuxTime1: int
    cameraTime1: int
    with PiCamera() as camera:
        linuxTime1 = int(time.time() * 10 ** 6)
        cameraTime1 = camera.timestamp
    # Get difference between two clocks
    timeDelta1 = linuxTime1 - cameraTime1

    logging.info(f"[OPNAV]: Recording from camera {camNum}")
    vidDataLow = record_video(
        OPNAV_MEDIA_DIR + f"cam{camNum}_expLow.mjpeg",
        framerate=camera_rec_params.fps,
        recTime=camera_rec_params.recTime,
        exposure=camera_rec_params.expLow,
    )
    vidDataHigh = record_video(
        OPNAV_MEDIA_DIR + f"cam{camNum}_expHigh.mjpeg",
        framerate=camera_rec_params.fps,
        recTime=camera_rec_params.recTime,
        exposure=camera_rec_params.expHigh,
    )

    # Get Unix time after recording(in seconds floating point -> microseconds)
    # Get camera time (in microseconds)
    linuxTime2: int
    cameraTime2: int
    with PiCamera() as camera:
        linuxTime2 = int(time.time() * 10 ** 6)
        cameraTime2 = camera.timestamp
    # Get difference between two clocks
    timeDelta2 = linuxTime2 - cameraTime2

    timeDeltaAvg = (timeDelta1 + timeDelta2) / 2

    return vidDataLow, vidDataHigh, timeDeltaAvg


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


def get_ephemeris(observeStart, body):
    # current_time = datetime.utcfromtimestamp(observeStart * 10 ** -6)
    current_time = observeStart
    # observeStart = observeStart - timedelta(microseconds=11716 * 1000)
    observeStart = observeStart - 11.716
    init_au = None
    current_au = None
    if body == BodyEnum.Sun:
        init_au = get_sun(Time(observeStart, format="unix")).cartesian
        current_au = get_sun(
            Time(current_time.strftime("%Y-%m-%dT%H:%M:%S"), format="isot", scale="tdb")
        ).cartesian
    elif body == BodyEnum.Moon:
        init_au = get_moon(
            Time(observeStart.strftime("%Y-%m-%dT%H:%M:%S"), format="isot", scale="tdb")
        ).cartesian
        current_au = get_moon(
            Time(current_time.strftime("%Y-%m-%dT%H:%M:%S"), format="isot", scale="tdb")
        ).cartesian

    init = CartesianRepresentation([init_au.x, init_au.y, init_au.z], unit="km")
    current = CartesianRepresentation(
        [current_au.x, current_au.y, current_au.z], unit="km"
    )
    x = current.x.value
    y = current.y.value
    z = current.z.value
    vx = (x - init.x.value) / (current_time - observeStart).seconds
    vy = (y - init.y.value) / (current_time - observeStart).seconds
    vz = (z - init.z.value) / (current_time - observeStart).seconds

    logging.info(f"{body} pos: ", x, y, z)
    logging.info(f"{body} vel: ", vx, vy, vz)

    return x, y, z, vx, vy, vz
