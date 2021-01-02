import numpy as np
import time

def select_camera(id):
    """
    Selects camera for recording video
    [id]: id of camera
    """
    time.sleep(3)
    raise NotImplementedError("implement camera selection")

def record_video(exposure):
    """
    Records video from selected camera
    [exposure]: exposure level for camera
    """
    time.sleep(3)
    raise NotImplementedError("implement record_video")

def record_gyro(count):
    """
    Records angular velocity from gyroscope
    [count]: number of measurements to record
    @returns
    [measurements]: angular velocites read from gyro scope
        Format: shape (count, 3) where each row: (omegax, omegay, omegaz)
        TODO: Obtain correct units for ang vel
    """
    time.sleep(3)
    raise NotImplementedError("implement record_gyro")
