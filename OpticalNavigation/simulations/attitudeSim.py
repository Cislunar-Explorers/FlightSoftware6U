import pandas as pd
import numpy as np
import math
import os
from animations import LiveMultipleAttitudePlot, Live2DPlot
from sim import isOrthogonal, attitudeMatrix 
from tests.gen_opnav_data import generateSyntheticData
import argparse

### FOR CAMERA ORIENTATION ###
from scipy.linalg import expm, norm
from numpy import cross, eye, dot
import math 

def M(axis, theta):
    """
    [axis]: [...] 3D list
    [theta]: float in radians
    """
    return expm(cross(eye(3), axis/norm(axis)*theta))

def rotateVector(vector, axis, theta):
    """
    [vector]: [...] 3D list
    [axis]: [...] 3D list
    [theta]: float in degrees
    """
    return dot(M(axis, math.radians(theta)),vector)

def getCameraVectors(posY, posZ):
    """
    posy = [...] positive body Y axis in target frame
    posz = [...] positive body Z axis in target frame
    """
    camera2vector = posY
    camera1vector = rotateVector(camera2vector, posZ, -60)
    camera3vector = rotateVector(camera2vector, posZ, 60)
    return camera1vector, camera2vector, camera3vector
##############################

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-a", "--kickduration", help = "how long is the cold gas thruster firing")
    args = vars(ap.parse_args())

    quat = np.array([[0., 0., 0., 1]]).T
    cameradt = 60 # seconds
    coldGasThrustKickTime = 20 # seconds
    coldGasKickDuration = float(args['kickduration'])
    omegaInit = [2, 0.1, 0.1, 0., 0., 0.]
    biasInit=[0., 0., 0.]
    gyroSampleCount = 4.

    gyroNoiseSigma = 1.e-7
    gyroSigma = 1.e-10
    gyro_t = 1.0 / gyroSampleCount
    # meas_sigma = 8.7e-4
    startTime = 0
    endTime = 200
    totalIntegrationTime = endTime - startTime

    q1, q2, q3, q4, omegax, omegay, omegaz, biasx, biasy, biasz = generateSyntheticData(quat, cameradt, coldGasThrustKickTime, coldGasKickDuration, omegaInit, biasInit, 1.0/gyroSampleCount, totalIntegrationTime, gyroSigma, gyroNoiseSigma)
    
    liveAtt = LiveMultipleAttitudePlot(bounds=[-1, 1, -1, 1, -1, 1])
    liveOmegas = Live2DPlot()

    arrow1 = liveAtt.addArrowFromOrigin(color='r', label='X')
    arrow2 = liveAtt.addArrowFromOrigin(color='g', label='Y')
    arrow3 = liveAtt.addArrowFromOrigin(color='b', label='Z')

    omx_id = liveOmegas.addGraph(color='r', label='omega x', alpha=0.5, ls='-', traceLim=50)
    omy_id = liveOmegas.addGraph(color='g', label='omega y', alpha=0.5, ls='-', traceLim=50)
    omz_id = liveOmegas.addGraph(color='b', label='omega z', alpha=0.5, ls='-', traceLim=50)

    camera1 = liveAtt.addArrowFromOrigin(color='orange', label='Cam_1', size=30, ls='--', alpha=0.25)
    camera2 = liveAtt.addArrowFromOrigin(color='orange', label='Cam_2', size=30, ls='--', alpha=0.25)
    camera3 = liveAtt.addArrowFromOrigin(color='orange', label='Cam_3', size=30, ls='--', alpha=0.25)

    camera1_2 = liveAtt.addArrowFromOrigin(color='orange', label='Cam_1+', size=30, ls='--', alpha=0.25)
    camera2_2 = liveAtt.addArrowFromOrigin(color='orange', label='Cam_2+', size=30, ls='--', alpha=0.25)
    camera3_2 = liveAtt.addArrowFromOrigin(color='orange', label='Cam_3+', size=30, ls='--', alpha=0.25)

    time = np.arange(startTime, endTime, gyro_t)

    for t in time:
        # Extract quaternions
        q = np.array([float(q1(t)), float(q2(t)), float(q3(t)), float(q4(t))])
        assert abs(np.linalg.norm(q) - 1) < 1e-3
        Aq = attitudeMatrix(q)

        # X axis of spacecraft
        local_vector = np.array([1, 0, 0, 0]) # last 0 is padding
        X_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

        # Y axis of spacecraft
        local_vector = np.array([0, 1, 0, 0]) # last 0 is padding
        Y_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

        # Z axis of spacecraft
        local_vector = np.array([0, 0, 1, 0]) # last 0 is padding
        Z_A = np.dot(Aq,local_vector[:3].T.reshape(3,1))

        liveAtt.updateOriginArrow(arrow1, X_A[0,0], X_A[1,0], X_A[2,0])
        liveAtt.updateOriginArrow(arrow2, Y_A[0,0], Y_A[1,0], Y_A[2,0])
        liveAtt.updateOriginArrow(arrow3, Z_A[0,0], Z_A[1,0], Z_A[2,0])

        # Update cameras
        camera1vector, camera2vector, camera3vector = getCameraVectors(Y_A.flatten(), Z_A.flatten())

        liveAtt.updateOriginArrow(camera1, camera1vector[0], camera1vector[1], camera1vector[2])
        liveAtt.updateOriginArrow(camera2, camera2vector[0], camera2vector[1], camera2vector[2])
        liveAtt.updateOriginArrow(camera3, camera3vector[0], camera3vector[1], camera3vector[2])

        # Update cameras
        camera1vector = rotateVector(camera1vector, X_A.flatten(), 360/8.)
        camera2vector = rotateVector(camera2vector, X_A.flatten(), 360/8.)
        camera3vector = rotateVector(camera3vector, X_A.flatten(), 360/8.)

        liveAtt.updateOriginArrow(camera1_2, camera1vector[0], camera1vector[1], camera1vector[2])
        liveAtt.updateOriginArrow(camera2_2, camera2vector[0], camera2vector[1], camera2vector[2])
        liveAtt.updateOriginArrow(camera3_2, camera3vector[0], camera3vector[1], camera3vector[2])

        liveOmegas.updateGraph(omx_id, t, float(omegax(t)))
        liveOmegas.updateGraph(omy_id, t, float(omegay(t)))
        liveOmegas.updateGraph(omz_id, t, float(omegaz(t)))

        # XYZ should be orthonormal
        assert isOrthogonal(X_A, Y_A)[0] and isOrthogonal(X_A, Z_A)[0] and isOrthogonal(Y_A, Z_A)[0] 

        liveAtt.render(text="{}s/{}s".format(round(t, 2),endTime),delay=0.001)
        liveOmegas.render(text="{}s/{}s".format(round(t, 2),endTime),delay=0.001)

    liveAtt.close()
    liveOmegas.close()

