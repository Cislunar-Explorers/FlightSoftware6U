import numpy as np
import time

from core.const import (
    ACQUISITION_ANGLE_INCREMENT,
    ACQUISITION_COMPENSATION_ROTATION,
    ACQUISITION_START_ANGLE,
    ACQUISITION_ANGLE_DISPLACEMENT,
)

INIT_IMG_CAPTURE_TIME = 1.2  # seconds
INIT_READ_OMEGA_TIME = 1.5  # seconds


def readOmega():
    """
    Returns:
    angular velocity on spin axis (rad/sec)
    """
    time.sleep(INIT_READ_OMEGA_TIME)
    return 6


def captureImg(dir):
    # time starts on command execution, and finishes when
    # OS gives back control to program
    time.sleep(INIT_IMG_CAPTURE_TIME)
    print("Image acquired: " + str(time.time()))


def acquire_frames(func_captureframe, dir):
    currentAngle = ACQUISITION_START_ANGLE  # degrees
    delta = ACQUISITION_ANGLE_DISPLACEMENT  # degrees
    timeImg = INIT_IMG_CAPTURE_TIME  # seconds
    omega = readOmega() * 180 / np.pi
    # adjust the amount of spin consumed due to delay
    timeWait = delta / omega - timeImg
    while timeWait < 0:
        delta = (
            delta + ACQUISITION_COMPENSATION_ROTATION
        )  # satellite must rotate for a full revolution
        timeWait = delta / omega - timeImg
    print(
        "Acquisition will consume {} extra rotations per frame".format(int(delta / 360))
    )
    while currentAngle >= 0:
        # Wait for satellite to get into position
        time.sleep(timeWait)
        startTime = time.time()
        func_captureframe(dir)  # takes timeImg seconds
        elapsedTime = time.time() - startTime

        print(elapsedTime)

        currentAngle = currentAngle - ACQUISITION_ANGLE_INCREMENT
        timeWait = delta / omega - timeImg


def startAcquisition(dir):
    """
    Beginds acquisition algorithm
    [dir]: Directory of acquired images. Should have subfolders Camera1/, Camera2/, Camera3/
    """
    acquire_frames(captureImg, dir)
