from core.const import TOTAL_INTEGRATION_TIME, GYRO_SIGMA, GYRO_NOISE_SIGMA, NX, LAM, R, _a, _f
import numpy
from numpy.linalg import inv
from numpy.linalg import pinv
from scipy.stats import norm
from numpy import random

def crs(vector):
    first = vector[0][0]
    second = vector[1][0]
    third = vector[2][0]
    return numpy.array([[0., -third, second],
                        [third, 0., -first],
                        [-second, first, 0.]])

# Dynamics Propagation Equations
def formA(q):
    q1, q2, q3, q4 = q[0][0], q[1][0], q[2][0], q[3][0]
    return numpy.array([[1. - 2.*q2**2. - 2.*q3**2., 2.*q1*q2 - 2.*q3*q4, 2.*q1*q3 + 2.*q2*q4],
                        [2.*q1*q2 + 2.*q3*q4, 1.-2.*q1**2. - 2.*q3**2., 2.*q2*q3 - 2.*q1*q4],
                        [2.*q1*q3 - 2.*q2*q4, 2.*q1*q4 + 2.*q2*q3, 1.-2.*q1**2. - 2.*q2**2.]])

def meas_model(state, earthVec, moonVec, sunVec):
    A = formA(state)
    zearth = numpy.dot(A, earthVec)
    zmoon = numpy.dot(A, moonVec)
    zsun = numpy.dot(A, sunVec)
    zearth = numpy.concatenate((zearth, zmoon), axis=0)
    zearth = numpy.concatenate((zearth, zsun), axis=0)
    return zearth

def getGyroMeasurement(t, gyro_noise_sigma, ws, bs):
    """
    [gyro_noise_sigma] noise added to true gyro values
    """
    # print(t)
    omegax, omegay, omegaz = ws[0], ws[1], ws[2]
    biasx, biasy, biasz = bs[0], bs[1], bs[2]
    return (numpy.array([[float(omegax(t)), float(omegay(t)),
                          float(omegaz(t))]]).T + numpy.array([[biasx(t)],
                                                               [biasy(t)],
                                                               [biasz(t)]]) +
            numpy.random.randn(3,1)*gyro_noise_sigma)

def updateOmega(t, biasEst, ws, bs):
    return getGyroMeasurement(t, GYRO_SIGMA, ws, bs) - biasEst

def updateBeta(betakp):
    return betakp

def computePsi(t, biasEst, sample_rate, ws, bs):
    omegaplus = updateOmega(t, biasEst, ws, bs)
    return (numpy.sin(0.5*sample_rate*numpy.linalg.norm(omegaplus))/
            numpy.linalg.norm(omegaplus)) * omegaplus

def computeBigOmega(t, biasEst, sample_rate, ws, bs):
    psi = computePsi(t, biasEst, sample_rate, ws, bs)
    oneone = numpy.cos(0.5*sample_rate*numpy.linalg.norm(updateOmega(t, biasEst, ws, bs)))
    psicrs = crs(psi)
    lowerright = numpy.array([[numpy.cos(0.5*sample_rate*
                                         numpy.linalg.norm(updateOmega(t, biasEst, ws, bs)))]])
    upperleft = oneone*numpy.eye(3) - psicrs
    upperright = psi
    lowerleft = -1*psi.T
    top = numpy.concatenate((upperleft, upperright), axis=1)
    bottom = numpy.concatenate((lowerleft, lowerright), axis=1)
    return numpy.concatenate((top, bottom), axis=0)

def updateQuaternion(t, biasEst, sample_rate, qkp, ws, bs):
    bigO = computeBigOmega(t, biasEst, sample_rate, ws, bs)
    return numpy.dot(bigO, qkp)
  
def getCholesky(Pmat, Qmat):
    return numpy.linalg.cholesky(Pmat + Q)

def extractColumns(mat):
    cols = numpy.zeros((len(mat), int(NX), 1))
    for i in range(len(mat)):
        cols[i] = (numpy.array([mat[:,i]]).T)
    return cols

def generateSigmas(xhatkp, Phatkp):
    sigpoints = numpy.zeros((int(2*NX + 1), int(NX), 1))
    sigpoints[0] = numpy.array([[0., 0., 0., xhatkp[3][0], xhatkp[4][0], xhatkp[5][0]]]).T
    S = getCholesky(Phatkp, Q)
    si = extractColumns(S)
    counter = 1
    for i in si:
        sigpoints[counter] = (sigpoints[0] + numpy.sqrt(NX+LAM)*i)
        sigpoints[counter+1] = (sigpoints[0] - numpy.sqrt(NX+LAM)*i)
        counter+=2
    return sigpoints

def makeErrorQuaternion(sigmas):
    errorquat = numpy.zeros((len(sigmas), 4, 1))
    counter = 0
    for i in sigmas:
        deltap = numpy.array([[i[0][0], i[1][0], i[2][0]]]).T
        deltaq4p = ((-_a*numpy.linalg.norm(deltap)**2. + 
                     _f*numpy.sqrt(_f**2. + (1-_a**2.)*numpy.linalg.norm(deltap)**2.))/
                    (_f**2. + numpy.linalg.norm(deltap)**2.))
        deltaqhatkp = (1./_f)*(_a + deltaq4p)*deltap
        deltaq = numpy.concatenate((deltaqhatkp, numpy.array([[deltaq4p]])), axis=0)
        deltaq = deltaq/numpy.linalg.norm(deltaq)
        errorquat[counter] = deltaq
        counter += 1
    return errorquat

def quaternionComposition(firstq, secondq):
    q1, q2, q3, q4 = firstq[0][0], firstq[1][0], firstq[2][0], firstq[3][0]
    mat = numpy.array([[q4, q3, -q2, q1],
                       [-q3, q4, q1, q2],
                       [q2, -q1, q4, q3],
                       [-q1, -q2, -q3, q4]])
    return numpy.dot(mat, secondq)

def perturbQuaternionEstimate(errorquat, qhatk):
    newquats = numpy.zeros((len(errorquat), 4, 1))
    counter = 0
    for i in errorquat:
        new = quaternionComposition(i, qhatk)
        new = new/numpy.linalg.norm(new)
        newquats[counter] = (new)
        counter += 1
    return newquats

def propagateQuaternion(perturbed_quat_list, sigmas, t, ws, bs, cameradt):
    time = numpy.arange(t, t+cameradt, GYRO_SAMPLE_RATE)
    # print(time)
    updated_quats = numpy.zeros((len(perturbed_quat_list), 4, 1))
    for i in range(len(perturbed_quat_list)):
        bias = numpy.array([[sigmas[i][3][0], sigmas[i][4][0], sigmas[i][5][0]]]).T
        quat = perturbed_quat_list[i]
        for j in time:
            newq = updateQuaternion(j, bias, GYRO_SAMPLE_RATE, quat, ws, bs)
            newq = newq/numpy.linalg.norm(newq)
            quat = newq
        updated_quats[i] = (quat)
    return updated_quats

def quaternionInv(quat):
    return numpy.array([[-1*quat[0][0]],
                        [-1*quat[1][0]],
                        [-1*quat[2][0]],
                        [ 1*quat[3][0]]])

def propagatedQuaternionError(newquats):
    errorprop = numpy.zeros((len(newquats), 4, 1))
    invquat = quaternionInv(newquats[0])
    counter = 0
    for i in newquats:
        err = quaternionComposition(i, invquat)
        err = err/numpy.linalg.norm(err)
        errorprop[counter] = (err)
        counter += 1
    return errorprop

def recoverPropSigma(quat_errors, old_sigmas):
    newsigs = numpy.zeros((len(quat_errors), int(NX), 1))
    for i in range(len(quat_errors)):
        deltaqhat = numpy.array([[quat_errors[i][0][0]],
                                 [quat_errors[i][1][0]],
                                 [quat_errors[i][2][0]]])
        deltaq4 = quat_errors[i][3][0]
        top = (_f*deltaqhat)/(_a + deltaq4)
        bottom = numpy.array([[old_sigmas[i][3][0]],
                              [old_sigmas[i][4][0]],
                              [old_sigmas[i][5][0]]])
        sig = numpy.concatenate((top, bottom), axis=0)
        newsigs[i]=(sig)
    return newsigs

def predictedMean(newsigs):
    xmean = (1./(NX + LAM))*(LAM*newsigs[0])
    for i in newsigs[1:]:
        xmean += (1./(2.*(NX+LAM)))*i
    return xmean

def predictedCov(newsigs, pred_mean):
    P = ((LAM/(NX + LAM)))*numpy.dot((newsigs[0] - pred_mean),(newsigs[0] - pred_mean).T)
    for i in newsigs[1:]:
        P += (1./(2*(NX+LAM)))*numpy.dot((i - pred_mean), (i - pred_mean).T)
    return P + Q

def hFromSigs(prop_quaternions, e, m, s):
    hlist = numpy.zeros((len(prop_quaternions), 9, 1))
    counter = 0
    for i in prop_quaternions:
        hlist[counter] = (meas_model(i, e, m, s))
        counter +=1
    return hlist

def meanMeasurement(hlist):
    zmean = (1./(NX + LAM))*(LAM * hlist[0])
    for i in hlist[1:]:
        zmean += (1./(2*(NX+LAM)))*i
    firstden = numpy.sqrt(zmean[0][0]**2. + zmean[1][0]**2. + zmean[2][0]**2.)
    secondden = numpy.sqrt(zmean[3][0]**2. + zmean[4][0]**2. + zmean[5][0]**2.)
    thirdden = numpy.sqrt(zmean[6][0]**2. + zmean[7][0]**2. + zmean[8][0]**2.)
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

def Pzz(hlist, zmean):
    Pzz = ((LAM/(NX+LAM)))*numpy.dot(hlist[0] - zmean, (hlist[0] - zmean).T)
    for i in hlist[1:]:
        Pzz += (1./(2*(NX+LAM)))*numpy.dot(i-zmean, (i-zmean).T)
    return Pzz + R

def Pxz(siglist, xmean, hlist, zmean):
    Pxzmat = ((LAM/(NX+LAM)))*numpy.dot(siglist[0] - xmean, (hlist[0]-zmean).T)
    for i in range(1,len(siglist)):
        Pxzmat += (1./(2*(NX+LAM)))*numpy.dot(siglist[i]-xmean,(hlist[i]-zmean).T)
    return Pxzmat

def getGain(Pxzmat,Pzzmat):
    return numpy.dot(Pxzmat, pinv(Pzzmat))

def updateXhat(xmean, K, measurement, zmean):
    inner = measurement - zmean
    right = numpy.dot(K, inner)
    return xmean + right

def updatePhat(Pmean, K, Pzzmat):
    return Pmean - numpy.dot(numpy.dot(K, Pzzmat), K.T)

def updateQuaternionEstimate(xhatkp, qhatkm):
    i = xhatkp
    deltap = numpy.array([[i[0][0], i[1][0], i[2][0]]]).T
    deltaq4p = ((-_a*numpy.linalg.norm(deltap)**2. + 
                 _f*numpy.sqrt(_f**2. + (1-_a**2.)*numpy.linalg.norm(deltap)**2.))/
                (_f**2. + numpy.linalg.norm(deltap)**2.))
    deltaqhatkp = (1./_f)*(_a + deltaq4p)*deltap
    deltaq = numpy.concatenate((deltaqhatkp, numpy.array([[deltaq4p]])), axis=0)
    new = quaternionComposition(deltaq, qhatkm)
    return new/numpy.linalg.norm(new)

def resetSigmaSeed(xhatkp):
    return numpy.array([[0., 0., 0., xhatkp[3][0], xhatkp[4][0], xhatkp[5][0]]]).T

def UKF(cameradt, P0, x0, q0, omegax, omegay, omegaz, biasx, biasy, biasz, earthVec, moonVec, sunVec, measurements):
    Phist = numpy.zeros((int(TOTAL_INTEGRATION_TIME/cameradt), 6, 6))
    qhist = numpy.zeros((int(TOTAL_INTEGRATION_TIME/cameradt), 4, 1))
    xhist = numpy.zeros((int(TOTAL_INTEGRATION_TIME/cameradt), 6, 1))
    Phatkp = P0
    xhatkp = x0
    qhatkp = q0
    time = 0.
    counter = 0
    for i in measurements[0:]:
        # if counter % 10==0:
        #     print(counter)
        sigma_points = generateSigmas(xhatkp, Phatkp)
        err_quats = makeErrorQuaternion(sigma_points)
        pert_quats = perturbQuaternionEstimate(err_quats, qhatkp)
        prop_quats = propagateQuaternion(pert_quats, sigma_points, time, (omegax, omegay, omegaz), (biasx, biasy, biasz), cameradt)
        qhatkp1m = prop_quats[0]
        prop_err = propagatedQuaternionError(prop_quats)
        prop_sigmas = recoverPropSigma(prop_err, sigma_points)
        x_mean = predictedMean(prop_sigmas)
        p_mean = predictedCov(prop_sigmas, x_mean)
        h_list = hFromSigs(prop_quats, earthVec, moonVec, sunVec)
        z_mean = meanMeasurement(h_list)
        Pzz_kp1 = Pzz(h_list, z_mean)
        Pxz_kp1 = Pxz(prop_sigmas, x_mean, h_list, z_mean)
        K = getGain(Pxz_kp1, Pzz_kp1)
        xhat_kp1 = updateXhat(x_mean, K, i, z_mean)
        Phat_kp1 = updatePhat(p_mean, K, Pzz_kp1)
        qhat_kp1 = updateQuaternionEstimate(xhat_kp1, qhatkp1m)
        
        Phist[counter] = Phat_kp1
        qhist[counter] = qhat_kp1
        xhist[counter] = xhat_kp1
        
        Phatkp = Phat_kp1
        xhatkp = resetSigmaSeed(xhat_kp1)
        qhatkp = qhat_kp1
        
        time += 1.
        counter += 1
        
    return Phist, qhist, xhist

def runAttitudeUKFWithKick(cameradt, omegax, omegay, omegaz, biasx, biasy, biasz, earthVec, moonVec, sunVec, measurements, x0, quat, P0, kickTime):
    # Obtain quats
    q0 = quat/numpy.linalg.norm(quat)
    # Run UKF
    return UKF(cameradt, P0, x0, q0, omegax, omegay, omegaz, biasx, biasy, biasz, earthVec, moonVec, sunVec, measurements)