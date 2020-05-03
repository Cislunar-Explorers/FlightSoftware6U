from core.acquisition import startAcquisition, readOmega
from core.cam_meas import cameraMeasurements
from core.ukf import runPosVelUKF
from core.attitude import runAttitudeUKFWithKick
import numpy as np
import traceback

"""
Begin OpNav processing sequence for obtaining position, velocity and attitude estimates.
This function is triggered once the "observer" mode of spacecraft has finished accumulating
enough data from its cameras and gyroscope. The input will be the start and end time for
the duration of the observer mode being on, ephemiris tables, starting states, camera
measurements, gyroscope measurements, main thruster fire sequences and cold-gas thruster 
fire sequences at specified times. The output will be the current position, velocity, 
attitude and angular velocity estimates. 
"""

# def run(currentTime, moonEph, sunEph, initState, P, cameraParameters, dir=None):
#     """
#     Executes OpNav pipeline once
#     [currentTime]: index into ephemiris table
#     [dir]: Directory of acquired images. Should have subfolders Camera1/, Camera2/, Camera3/
#     [cameraParameters]: camera settings used to take the pictures
#     Ephemiris
#     Returns:
#     [xNew, P, K]: OpNav finished successfully
#     [None, None, None]: Measurements did not find all three bodies
#     Throws:
#     Exception if measurements failed
#     """
#     print("Starting OpNav")
#     #startAcquisition(dir)
#     try:
#         meas = cameraMeasurements([0, 0, readOmega()], 0.91, dir, cameraParameters)
#         if meas is None:
#             print("[Opnav controller]: did not find all three bodies. Skipping...")
#             return None, None, None
#         else:
#             xNew, pNew, K = runPosVelUKF(moonEph[currentTime].reshape(1,6), sunEph[currentTime].reshape(1,6), meas.reshape(6,1), initState, 60, P) # TODO: UKF computation errors?
#             # TODO: Deposit position and velocity estimates into global location
#             print("[Opnav controller]: Finished successfully {}, Gain {}".format(xNew, K))
#             return xNew, pNew, K
#     except Exception as e:
#         traceback.print_stack()
#         raise Exception("[Opnav controller]: Error while trying to compute measurements: {}".format(e))
