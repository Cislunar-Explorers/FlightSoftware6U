from core.const import (
    BodyEnum,
    CisLunarCameraParameters,
    FileData,
    DetectionData,
    Vector3,
    CameraParameters,
    CameraRecordingParameters,
)
from core.find_algos.find_with_contours import find
from core.sense import select_camera, record_video

from adafruit_blinka.agnostic import board_id

if board_id and board_id != "GENERIC_LINUX_PC":
    from picamera import PiCamera

from utils.constants import OPNAV_MEDIA_DIR

import numpy as np
import math
import time
import logging
from typing import Tuple
from astropy.time import Time
from astropy.coordinates import get_sun, get_moon, CartesianRepresentation


def record_with_timedelta(
    camNum: int, camera_rec_params: CameraRecordingParameters = CisLunarCameraParameters
) -> Tuple[Tuple[str, "list[int]"], Tuple[str, "list[int]"], "list[int]"]:
    """
    Outputs:
    vidDataLow, vidDataHigh: tuples of video filename (str) and list to timestamps (int)
    timeDeltaAvg: difference between camera clock timestamps at two times (int, usec)
    """

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


def get_detections(frames: "list[str]") -> "list[DetectionData]":
    logging.info("[OPNAV]: Finding...")
    progress = 1
    detections = []
    for f in range(len(frames)):
        logging.info(f"[OPNAV]: Image {progress}/{len(frames)}: {frames[f]}")
        fileInfo = FileData(frames[f])
        detectedBodies = find(frames[f])

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


def get_best_detection(detections: "list[DetectionData]") -> DetectionData:
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


def cam_to_body(
    detection: "list[DetectionData]", camera_params: CameraParameters
) -> DetectionData:
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


def tZeroRotMatrix(rotation: float) -> np.ndarray:
    """Creates a y-axis rotation matrix, from rotation rate (rad/s)"""
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


def get_elapsed_time(
    fileData: FileData, timeDeltaAvgs: "list[int]", observeStart: int
) -> float:
    """
    Calculates the elapsed time (in seconds, floating pt) between the beginning of the opnav observe call and the
    timestamp of a selected frame
    timeDeltaAvgs: microseconds
    """
    timestamp = fileData.timestamp
    camNum = fileData.cam_num
    timestampUnix = timestamp + timeDeltaAvgs[camNum - 1]
    timeElapsed = (timestampUnix - observeStart) * 10 ** -6  # Microseconds to seconds
    return timeElapsed


def body_to_T0(
    detection: "list[DetectionData]", timeElapsed: float, avgGyroY: float
) -> DetectionData:
    logging.info(f"[OPNAV]: {detection.detection} Time Elapsed= {timeElapsed}")
    rotation = avgGyroY * timeElapsed
    tZeroRotation = tZeroRotMatrix(rotation)
    res = (tZeroRotation).dot(detection.vector.data)
    detection.vector = Vector3(res[0], res[1], res[2])
    return detection


def get_ephemeris(observeStart: int, body: BodyEnum) -> float:
    # Astropy needs unix timestamp in seconds!!!
    current_time = observeStart
    observeStart = observeStart - 11.716
    init_au = None
    current_au = None
    if body == BodyEnum.Sun:
        init_au = get_sun(Time(observeStart, format="unix")).cartesian
        current_au = get_sun(Time(current_time, format="unix")).cartesian
    elif body == BodyEnum.Moon:
        init_au = get_moon(Time(observeStart, format="unix")).cartesian
        current_au = get_moon(Time(current_time, format="unix")).cartesian

    init = CartesianRepresentation([init_au.x, init_au.y, init_au.z], unit="km")
    current = CartesianRepresentation(
        [current_au.x, current_au.y, current_au.z], unit="km"
    )
    x = current.x.value
    y = current.y.value
    z = current.z.value
    vx = (x - init.x.value) / (current_time - observeStart)
    vy = (y - init.y.value) / (current_time - observeStart)
    vz = (z - init.z.value) / (current_time - observeStart)

    logging.info("Got 'em all!")
    logging.info(f"{body.name} pos:  {x}, {y}, {z}")
    logging.info(f"{body.name} vel: {vx}, {vy}, {vz}")

    return x, y, z, vx, vy, vz
