from core.acquisition import startAcquisition, readOmega
from core.cam_meas import cameraMeasurements
from core.ukf import runUKF
import numpy as np
import traceback

def run(currentTime, moonEph, sunEph, initState, P, cameraParameters, dir=None):
    """
    Executes OpNav pipeline once
    [currentTime]: index into ephemiris table
    [dir]: Directory of acquired images. Should have subfolders Camera1/, Camera2/, Camera3/
    [cameraParameters]: camera settings used to take the pictures
    Ephemiris
    Returns:
    [xNew, P, K]: OpNav finished successfully
    [None, None, None]: Measurements did not find all three bodies
    Throws:
    Exception if measurements failed
    """
    print("Starting OpNav")
    #startAcquisition(dir)
    try:
        meas = cameraMeasurements([0, 0, readOmega()], 0.91, dir, cameraParameters)
        if meas is None:
            print("[Opnav controller]: did not find all three bodies. Skipping...")
            return None, None, None
        else:
            xNew, pNew, K = runUKF(moonEph[currentTime].reshape(1,6), sunEph[currentTime].reshape(1,6), meas.reshape(6,1), initState, 60, P) # TODO: UKF computation errors?
            # TODO: Deposit position and velocity estimates into global location
            print("[Opnav controller]: Finished successfully {}, Gain {}".format(xNew, K))
            return xNew, pNew, K
    except Exception as e:
        traceback.print_stack()
        raise Exception("[Opnav controller]: Error while trying to compute measurements: {}".format(e))
