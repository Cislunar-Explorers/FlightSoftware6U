from core.acquisition import startAcquisition, readOmega
from core.cam_meas import cameraMeasurements
from core.ukf import runPosVelUKF
from core.attitude import runAttitudeUKFWithKick
import numpy as np
import traceback

"""
Begin OpNav acquisition and storing process. The system will record videos from
the three cameras onboard and store them on the SD card as video format. It will
record gyroscope measurements and store them in a table on the SD card.
"""
def observe():
    pass

"""
Begin OpNav processing sequence for obtaining position, velocity and attitude estimates.
This function is triggered once the "observer" mode of spacecraft has finished accumulating
enough data from its cameras and gyroscope. The input will be the start and end time for
the duration of the observer mode being on, ephemiris tables, starting states, camera
measurements, gyroscope measurements, main thruster fire sequences and cold-gas thruster 
fire sequences at specified times. The output will be the current position, velocity, 
attitude and angular velocity estimates. 
"""
def process(startTime, duration, moonTable, sunTable, initPosVel, initOmega, initQuat, camDir, gyroCSV):
    """
    [startTime] start time index for ephemeris tables
    [duration] how long the observation mode lasted
    [moonTable, sunTable] full ephemeris tables
    [initPosVel] previous position and velocity estimate
    [initOmega] previous angular velocity estimate
    [initQuat] previous orientation quaternion estimate
    [camDir] directory of the camera video clips
    [gyroCSV] location of the gyro measurements csv file
    Returns:
    [newPos, newVel, newOmega, newQuat]
    """
    # Step 1: Convert video clips into list of measurements
    # TODO: Finalize video format and conversion to frames
    measlist = [np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])]
    gyro_list = []

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
