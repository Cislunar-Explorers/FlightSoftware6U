from typing import List
from OpticalNavigation.core.const import AttitudeEstimateOutput, AttitudeStateVector, CovarianceMatrix, EphemerisVector, GyroMeasurementVector, GyroVars, QuaternionVector
from OpticalNavigation.core.const import AttitudeUKFConstants
import numpy as np
from numpy.linalg import inv
from numpy.linalg import pinv
from scipy.stats import norm
from numpy import random
from tqdm import tqdm

NX = AttitudeUKFConstants.NX
LAM = AttitudeUKFConstants.LAM
_a = AttitudeUKFConstants._a 
_f = AttitudeUKFConstants._f

def crs(vector):
    first = vector[0][0]
    second = vector[1][0]
    third = vector[2][0]
    return np.array([[0., -third, second],
                        [third, 0., -first],
                        [-second, first, 0.]])

# Dynamics Propagation Equations
def formA(q):
    q1, q2, q3, q4 = q[0][0], q[1][0], q[2][0], q[3][0]
    return np.array([[1. - 2.*q2**2. - 2.*q3**2., 2.*q1*q2 - 2.*q3*q4, 2.*q1*q3 + 2.*q2*q4],
                        [2.*q1*q2 + 2.*q3*q4, 1.-2.*q1**2. - 2.*q3**2., 2.*q2*q3 - 2.*q1*q4],
                        [2.*q1*q3 - 2.*q2*q4, 2.*q1*q4 + 2.*q2*q3, 1.-2.*q1**2. - 2.*q2**2.]])

def meas_model(state, earthVec, moonVec, sunVec):
    A = formA(state)
    zearth = np.dot(A, earthVec)
    zmoon = np.dot(A, moonVec)
    zsun = np.dot(A, sunVec)
    zearth = np.concatenate((zearth, zmoon), axis=0)
    zearth = np.concatenate((zearth, zsun), axis=0)
    return zearth

def getGyroMeasurement(j, gyro_noise_sigma, ws, bs):
    """
    [gyro_noise_sigma] noise added to true gyro values
    [j]: index of current gyro measurement (0...gyroSampleCount)
    """
    # omegax, omegay, omegaz = ws[0], ws[1], ws[2]
    # biasx, biasy, biasz = bs[0], bs[1], bs[2]
    return (ws[j].data.reshape(3,1)) + (bs.reshape(3,1)) + np.random.randn(3,1)*gyro_noise_sigma

def updateOmega(j, biasEst, gyro_sigma, ws, bs):
    return getGyroMeasurement(j, gyro_sigma, ws, bs) - biasEst

def updateBeta(betakp):
    return betakp

def computePsi(j, biasEst, gyro_sigma, sample_rate, ws, bs):
    omegaplus = updateOmega(j, biasEst, gyro_sigma, ws, bs)
    return (np.sin(0.5*sample_rate*np.linalg.norm(omegaplus))/
            np.linalg.norm(omegaplus)) * omegaplus

def computeBigOmega(j, biasEst, gyro_sigma, sample_rate, ws, bs):
    psi = computePsi(j, biasEst, gyro_sigma, sample_rate, ws, bs)
    oneone = np.cos(0.5*sample_rate*np.linalg.norm(updateOmega(j, biasEst, gyro_sigma, ws, bs)))
    psicrs = crs(psi)
    lowerright = np.array([[np.cos(0.5*sample_rate*
                                         np.linalg.norm(updateOmega(j, biasEst, gyro_sigma, ws, bs)))]])
    upperleft = oneone*np.eye(3) - psicrs
    upperright = psi
    lowerleft = -1*psi.T
    top = np.concatenate((upperleft, upperright), axis=1)
    bottom = np.concatenate((lowerleft, lowerright), axis=1)
    return np.concatenate((top, bottom), axis=0)

def updateQuaternion(j, biasEst, gyro_sigma, sample_rate, qkp, ws, bs):
    """
    [j]: time index in current time segment
    """
    bigO = computeBigOmega(j, biasEst, gyro_sigma, sample_rate, ws, bs)
    return np.dot(bigO, qkp)
  
def getCholesky(Pmat, Qmat):
    return np.linalg.cholesky(Pmat + Qmat)

def extractColumns(mat):
    cols = np.zeros((len(mat), int(NX), 1))
    for i in range(len(mat)):
        cols[i] = (np.array([mat[:,i]]).T)
    return cols

def generateSigmas(xhatkp, Phatkp, Q):
    sigpoints = np.zeros((int(2*NX + 1), int(NX), 1))
    sigpoints[0] = np.array([[0., 0., 0., xhatkp[3][0], xhatkp[4][0], xhatkp[5][0]]]).T
    S = getCholesky(Phatkp, Q)
    si = extractColumns(S)
    counter = 1
    for i in si:
        sigpoints[counter] = (sigpoints[0] + np.sqrt(NX+LAM)*i)
        sigpoints[counter+1] = (sigpoints[0] - np.sqrt(NX+LAM)*i)
        counter+=2
    return sigpoints

def makeErrorQuaternion(sigmas):
    errorquat = np.zeros((len(sigmas), 4, 1))
    counter = 0
    for i in sigmas:
        deltap = np.array([[i[0][0], i[1][0], i[2][0]]]).T
        deltaq4p = ((-_a*np.linalg.norm(deltap)**2. + 
                     _f*np.sqrt(_f**2. + (1-_a**2.)*np.linalg.norm(deltap)**2.))/
                    (_f**2. + np.linalg.norm(deltap)**2.))
        deltaqhatkp = (1./_f)*(_a + deltaq4p)*deltap
        deltaq = np.concatenate((deltaqhatkp, np.array([[deltaq4p]])), axis=0)
        deltaq = deltaq/np.linalg.norm(deltaq)
        errorquat[counter] = deltaq
        counter += 1
    return errorquat

def quaternionComposition(firstq, secondq):
    q1, q2, q3, q4 = firstq[0][0], firstq[1][0], firstq[2][0], firstq[3][0]
    mat = np.array([[q4, q3, -q2, q1],
                       [-q3, q4, q1, q2],
                       [q2, -q1, q4, q3],
                       [-q1, -q2, -q3, q4]])
    return np.dot(mat, secondq)

def perturbQuaternionEstimate(errorquat, qhatk):
    newquats = np.zeros((len(errorquat), 4, 1))
    counter = 0
    for i in errorquat:
        new = quaternionComposition(i, qhatk)
        new = new/np.linalg.norm(new)
        newquats[counter] = (new)
        counter += 1
    return newquats

def propagateQuaternion(perturbed_quat_list, sigmas, time_segment, gyro_sigma, gyro_sample_rate, ws, bs, cameradt):
    """
    [time_segment]: time deltas of gyro readings: [avg delta t, 2nd-1st, 3rd-2nd, ...]
    [ws,bs]: omegas and biases obtained from [time_segment]
    """
    #time = np.arange(t, t+cameradt, gyro_sample_rate)
    updated_quats = np.zeros((len(perturbed_quat_list), 4, 1))
    for i in range(len(perturbed_quat_list)):
        bias = np.array([[sigmas[i][3][0], sigmas[i][4][0], sigmas[i][5][0]]]).T
        quat = perturbed_quat_list[i]
        for j, t in enumerate(time_segment):
            newq = updateQuaternion(j, bias, gyro_sigma, gyro_sample_rate, quat, ws, bs)
            newq = newq/np.linalg.norm(newq)
            quat = newq
        updated_quats[i] = (quat)
    return updated_quats

def quaternionInv(quat):
    return np.array([[-1*quat[0][0]],
                        [-1*quat[1][0]],
                        [-1*quat[2][0]],
                        [ 1*quat[3][0]]])

def propagatedQuaternionError(newquats):
    errorprop = np.zeros((len(newquats), 4, 1))
    invquat = quaternionInv(newquats[0])
    counter = 0
    for i in newquats:
        err = quaternionComposition(i, invquat)
        err = err/np.linalg.norm(err)
        errorprop[counter] = (err)
        counter += 1
    return errorprop

def recoverPropSigma(quat_errors, old_sigmas):
    newsigs = np.zeros((len(quat_errors), int(NX), 1))
    for i in range(len(quat_errors)):
        deltaqhat = np.array([[quat_errors[i][0][0]],
                                 [quat_errors[i][1][0]],
                                 [quat_errors[i][2][0]]])
        deltaq4 = quat_errors[i][3][0]
        top = (_f*deltaqhat)/(_a + deltaq4)
        bottom = np.array([[old_sigmas[i][3][0]],
                              [old_sigmas[i][4][0]],
                              [old_sigmas[i][5][0]]])
        sig = np.concatenate((top, bottom), axis=0)
        newsigs[i]=(sig)
    return newsigs

def predictedMean(newsigs):
    xmean = (1./(NX + LAM))*(LAM*newsigs[0])
    for i in newsigs[1:]:
        xmean += (1./(2.*(NX+LAM)))*i
    return xmean

def predictedCov(newsigs, pred_mean, Q):
    P = ((LAM/(NX + LAM)))*np.dot((newsigs[0] - pred_mean),(newsigs[0] - pred_mean).T)
    for i in newsigs[1:]:
        P += (1./(2*(NX+LAM)))*np.dot((i - pred_mean), (i - pred_mean).T)
    return P + Q

def hFromSigs(prop_quaternions, e, m, s):
    hlist = np.zeros((len(prop_quaternions), 9, 1))
    counter = 0
    for i in prop_quaternions:
        hlist[counter] = meas_model(i, e, m, s).reshape(9, 1)
        counter +=1
    return hlist

def meanMeasurement(hlist):
    zmean = (1./(NX + LAM))*(LAM * hlist[0])
    for i in hlist[1:]:
        zmean += (1./(2*(NX+LAM)))*i
    firstden = np.sqrt(zmean[0][0]**2. + zmean[1][0]**2. + zmean[2][0]**2.)
    secondden = np.sqrt(zmean[3][0]**2. + zmean[4][0]**2. + zmean[5][0]**2.)
    thirdden = np.sqrt(zmean[6][0]**2. + zmean[7][0]**2. + zmean[8][0]**2.)
    zmean[0][0] = zmean[0][0]/firstden
    zmean[1][0] = zmean[1][0]/firstden
    zmean[2][0] = zmean[2][0]/firstden
    zmean[3][0] = zmean[3][0]/secondden
    zmean[4][0] = zmean[4][0]/secondden
    zmean[5][0] = zmean[5][0]/secondden
    zmean[6][0] = zmean[6][0]/thirdden
    zmean[7][0] = zmean[7][0]/thirdden
    zmean[8][0] = zmean[8][0]/thirdden
    return zmean

def Pzz(hlist, zmean, R):
    Pzz = ((LAM/(NX+LAM)))*np.dot(hlist[0] - zmean, (hlist[0] - zmean).T)
    for i in hlist[1:]:
        Pzz += (1./(2*(NX+LAM)))*np.dot(i-zmean, (i-zmean).T)
    return Pzz + R

def Pxz(siglist, xmean, hlist, zmean):
    Pxzmat = ((LAM/(NX+LAM)))*np.dot(siglist[0] - xmean, (hlist[0]-zmean).T)
    for i in range(1,len(siglist)):
        Pxzmat += (1./(2*(NX+LAM)))*np.dot(siglist[i]-xmean,(hlist[i]-zmean).T)
    return Pxzmat

def getGain(Pxzmat,Pzzmat):
    return np.dot(Pxzmat, pinv(Pzzmat))

def updateXhat(xmean, K, measurement, zmean):
    inner = measurement - zmean
    right = np.dot(K, inner)
    return xmean + right

def updatePhat(Pmean, K, Pzzmat):
    return Pmean - np.dot(np.dot(K, Pzzmat), K.T)

def updateQuaternionEstimate(xhatkp, qhatkm):
    i = xhatkp
    deltap = np.array([[i[0][0], i[1][0], i[2][0]]]).T
    deltaq4p = ((-_a*np.linalg.norm(deltap)**2. + 
                 _f*np.sqrt(_f**2. + (1-_a**2.)*np.linalg.norm(deltap)**2.))/
                (_f**2. + np.linalg.norm(deltap)**2.))
    deltaqhatkp = (1./_f)*(_a + deltaq4p)*deltap
    deltaq = np.concatenate((deltaqhatkp, np.array([[deltaq4p]])), axis=0)
    new = quaternionComposition(deltaq, qhatkm)
    return new/np.linalg.norm(new)

def resetSigmaSeed(xhatkp):
    return np.array([[0., 0., 0., xhatkp[3][0], xhatkp[4][0], xhatkp[5][0]]]).T

def UKFSingle(cameradt: float, gyroVars: GyroVars, P0: np.ndarray,
              x0: np.ndarray, q0: np.ndarray, omegas,
              estimatedSatState, moonEph: np.ndarray,
              sunEph: np.ndarray, timeline: List[float]) -> AttitudeEstimateOutput:
    """
    NOTE: Does not make any assumptions about gyroSampleCount
    let n = # of camera measurements in batch
    let gyroSampleCount = 1/gyro_sample_rate
    [gyroVars]: (gyro_sigma, gyro_sample_rate, Q, R)
    [omegas]: (x, y, z) components of measured angular velocity (n x gyroSampleCount, 3) 
    [estimatedSatState]: trajectory UKF outputs from t = 0 to t = totalIntegrationTime (n x gyroSampleCount, 3)
    [moonEph]: moon position/vel from ephemeris table (n x gyroSampleCount, 3)
    [sunEph]: sun position/vel from ephemeris table (n x gyroSampleCount, 3)
    [timeline]: time deltas of gyro readings: [avg delta t, 2nd-1st, 3rd-2nd, ...]
    See runAttitudeUKF() for remaining parameters and return type info.
    """
    # assert(omegas[0].shape[0] == biases.shape[0] == estimatedSatState.shape[0] == moonEph.shape[0] == sunEph.shape[0])
    n = moonEph.shape[0]
    (gyro_sigma, gyro_sample_rate, Q, R) = gyroVars

    # Phist = np.zeros((int(n/gyroSampleCount), 6, 6))
    # qhist = np.zeros((int(n/gyroSampleCount), 4, 1))
    # xhist = np.zeros((int(n/gyroSampleCount), 6, 1))
    Phatkp = P0
    xhatkp = x0
    qhatkp = q0

    # Calculate position vectors
    # satPos, moonPos, sunPos = estimatedSatState[i:i+gyroSampleCount,:], moonEph[i:i+gyroSampleCount,:], sunEph[i:i+gyroSampleCount,:]
    # earthVec, moonVec, sunVec = np.zeros((gyroSampleCount, 3)), np.zeros((gyroSampleCount, 3)), np.zeros((gyroSampleCount, 3))
    # for g_sample in range(gyroSampleCount):
        # earthVec[i+g_sample] = (-1*satPos[i+g_sample])/np.linalg.norm(satPos[i+g_sample])
        # moonVec[i+g_sample] = (moonPos[i+g_sample] - satPos[i+g_sample])/np.linalg.norm(moonPos[i+g_sample] - satPos[i+g_sample])
        # sunVec[i+g_sample] = (sunPos[i+g_sample] - satPos[i+g_sample])/np.linalg.norm(sunPos[i+g_sample] - satPos[i+g_sample])
    satPos, moonPos, sunPos = estimatedSatState[0,:], moonEph[0,:], sunEph[0,:]
    earthVec = (-1*satPos)/np.linalg.norm(satPos)
    moonVec = (moonPos - satPos)/np.linalg.norm(moonPos - satPos)
    sunVec = (sunPos - satPos)/np.linalg.norm(sunPos - satPos)
    sigma_points = generateSigmas(xhatkp, Phatkp, Q)
    err_quats = makeErrorQuaternion(sigma_points)
    pert_quats = perturbQuaternionEstimate(err_quats, qhatkp)
    prop_quats = propagateQuaternion(pert_quats, sigma_points, timeline, gyro_sigma, gyro_sample_rate, omegas, x0[3:], cameradt)
    qhatkp1m = prop_quats[0]
    prop_err = propagatedQuaternionError(prop_quats)
    prop_sigmas = recoverPropSigma(prop_err, sigma_points)
    x_mean = predictedMean(prop_sigmas)
    p_mean = predictedCov(prop_sigmas, x_mean, Q)
    h_list = hFromSigs(prop_quats, earthVec, moonVec, sunVec)
    z_mean = meanMeasurement(h_list)
    Pzz_kp1 = Pzz(h_list, z_mean, R)
    Pxz_kp1 = Pxz(prop_sigmas, x_mean, h_list, z_mean)
    K = getGain(Pxz_kp1, Pzz_kp1)
    meas = np.concatenate((earthVec, moonVec, sunVec), axis=0).reshape(9, 1)
    assert(z_mean.shape == meas.shape)
    xhat_kp1 = updateXhat(x_mean, K, meas, z_mean)
    Phat_kp1 = updatePhat(p_mean, K, Pzz_kp1)
    qhat_kp1 = updateQuaternionEstimate(xhat_kp1, qhatkp1m)
    
    # Phist[counter] = Phat_kp1
    # qhist[counter] = qhat_kp1
    # xhist[counter] = xhat_kp1
    
    # Phatkp = Phat_kp1
    # xhatkp = resetSigmaSeed(xhat_kp1)
    # qhatkp = qhat_kp1
        
    return AttitudeEstimateOutput(new_state=AttitudeStateVector.from_numpy_array(state=xhat_kp1), 
                                new_P=CovarianceMatrix(matrix=Phat_kp1), 
                                new_quat=QuaternionVector.from_numpy_array(quat=qhat_kp1))


def runAttitudeUKF(cameradt: float, gyroVars: GyroVars, P0: CovarianceMatrix,
                   x0: AttitudeStateVector, quat: QuaternionVector, omegas: List[GyroMeasurementVector],
                   satState, moonEph: np.ndarray,
                   sunEph: np.ndarray, timeline: List[float], singleIteration=False) -> AttitudeEstimateOutput:
    """
    Runs the attitude UKF and produces attitude estimates in the form of:
        1) attitde error state (Rodriguez Parameters) + bias (size 6x1)
        2) quaternion attitude state (size 4x1)
    See "notes/Full Attitude UKF.html" for detailed explaination and theoritical background on this UKF.
    [cameradt]: time since last camera run
    [gyroVars]: (gyro_sigma, gyro_sample_rate, Q, R) where
                [gyro_sigma]: float
                [gyro_sample_rate]: average sample time elapsed between each gyro measurement (seconds)
                [Q]: (6,6) np matrix
                [R]: (6,6) np matrix
    [P0]: initial covariance matrix for state estimate
    [x0]: (6,1) np array initial attitude State
    [quat]: (4x1) quaternion state (if this first iteration, this can be random as it is derived from [x0])
    [omegas]: (x, y, z) components of measured angular velocity (n x gyroSampleCount, 3) 
    [estimatedSatState]: trajectory UKF outputs from t = 0 to t = totalIntegrationTime (n x gyroSampleCount, 3)
    [moonEph]: moon position/vel from ephemeris table (n x gyroSampleCount, 3)
    [sunEph]: sun position/vel from ephemeris table (n x gyroSampleCount, 3)
    [timeline]: time deltas of omegas and biases readings: [avg delta t, 2nd-1st, 3rd-2nd, ...]
    [singleIteration]: [True] run single iteration [False] run batch iteration 
    returns:
    [Phat_kp1]: new covariance matrix for state estimate (6x6)
    [qhat_kp1]: estimated quaternion state (4x1)
    [xhat_kp1]: estimated attitude error state (6x1)
    If [singleIteration] was set to False, then estimates computed at the final interation of the batch are returned.
    """
    # assert cameradt >= 0.
    # assert len(gyroVars) == 4
    # assert gyroVars[2].shape == (6,6)
    # assert gyroVars[3].shape == (9,9)
    # assert P0.shape == (6,6)
    # assert x0.shape == (6,1)
    # assert quat.shape == (4,1)
    quat.data = quat.data/np.linalg.norm(quat.data)
    # if singleIteration is False:
    #     return UKFMultiple(cameradt, gyroVars, P0, x0, q0, omegas, biases, satState, moonEph, sunEph, timeline)
    return UKFSingle(cameradt, gyroVars, P0.data.reshape(6,6), x0.data.reshape(6,1), quat.data.reshape(4,1), omegas, satState, moonEph, sunEph, timeline)