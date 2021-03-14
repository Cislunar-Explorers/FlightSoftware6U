import numpy as np
import time
from OpticalNavigation.core.camera import Camera, CameraMux

def select_camera(id):
    """
    Selects camera for recording video
    [id]: id of camera
    """
    mux = CameraMux()
    mux.selectCamera(id)

def record_video(filename, framerate, recTime, exposure):
    """
    Records video from selected camera
    [exposure]: exposure level for camera
    """
    cam = Camera()
    filename_timestamp = cam.rawObservation(filename, frame_rate=framerate, video_time=recTime, shutterspeed=exposure)
    return filename_timestamp
    
def record_gyro(count):
    """
    Records angular velocity from gyroscope
    [count]: number of measurements to record
    @returns
    [measurements]: angular velocites read from gyro scope
        Format: shape (count, 3) where each row: (omegax, omegay, omegaz)
        TODO: Obtain correct units for ang vel
    """
    # TODO: coordinate with Toby on integrating gyro from telemetry file into this
    #time.sleep(3)
    #raise NotImplementedError("implement record_gyro")
    pass
