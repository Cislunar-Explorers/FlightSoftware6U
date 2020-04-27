from const import SPACECRAFT_I_B, DAMPER_C, DAMPER_I_D, TOTAL_INTEGRATION_TIME, INTEGRATION_TIMESTEP, GYRO_SAMPLE_RATE, GYRO_SIGMA, GYRO_NOISE_SIGMA, BIAS_INIT, MEAS_SIGMA, NX, LAM, P0, Q, R, _a, _f

import matplotlib.pyplot as plt
import numpy
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
from numpy.linalg import pinv
from scipy.stats import norm
from scipy import integrate
from numpy import random
from copy import deepcopy
from scipy.interpolate import InterpolatedUnivariateSpline

def crs(vector):
    first = vector[0][0]
    second = vector[1][0]
    third = vector[2][0]
    return numpy.array([[0., -third, second],
                        [third, 0., -first],
                        [-second, first, 0.]])

def propagateSpacecraft(X, t, kickTime):
    omegasc1, omegasc2, omegasc3 = X[0], X[1], X[2]
    omegad1, omegad2, omegad3 = X[3], X[4], X[5]
    omega_sc = numpy.array([[omegasc1, omegasc2, omegasc3]]).T
    omega_d  = numpy.array([[omegad1, omegad2, omegad3]]).T
    
    inner_sc = (numpy.dot(crs(omega_sc), numpy.dot(SPACECRAFT_I_B, omega_sc)) - DAMPER_C*omega_d +
                    int(0.5*(numpy.sign(t - kickTime)+1))*numpy.array([[.1, 0., 0.]]).T -
                    int(0.5*(numpy.sign(t - (kickTime + 2))+1))*numpy.array([[.1, 0., 0.]]).T)
    omega_sc_dot = (numpy.dot(-1.*pinv(SPACECRAFT_I_B), inner_sc))
    
    right_d = numpy.dot(crs(omega_sc), numpy.dot(DAMPER_I_D, omega_d + omega_sc)) + DAMPER_C*omega_d
    inner_d = numpy.dot(DAMPER_I_D, omega_sc_dot) + right_d
    omega_d_dot = numpy.dot(-1.*pinv(DAMPER_I_D), inner_d)
    
    derivs = [omega_sc_dot[0][0], omega_sc_dot[1][0], omega_sc_dot[2][0],
              omega_d_dot[0][0], omega_d_dot[1][0], omega_d_dot[2][0]]
    
    return derivs

def goSpacecraft(X, tstop, delta_t, kickTime):
    a_t = numpy.arange(0, tstop, delta_t)
    asol = integrate.odeint(propagateSpacecraft, X, a_t, args=(kickTime,))
    hx, hy, hz = [], [], []
    htotx, htoty, htotz = [], [], []
    omegasx, omegasy, omegasz = [], [], []
    omegadx, omegady, omegadz = [], [], []
    h_norm_spacecraft = []
    hnorm = []
    for i in asol:
        omega_spacecraft = numpy.array([[i[0], i[1], i[2]]]).T
        omega_damper = numpy.array([[i[3], i[4], i[5]]]).T
        h = numpy.dot(SPACECRAFT_I_B, omega_spacecraft)
        h_norm_spacecraft.extend([numpy.linalg.norm(h)])
        hdamp = numpy.dot(DAMPER_I_D, omega_damper + omega_spacecraft)
        htot = h + hdamp
        hnorm.extend([numpy.linalg.norm(htot)])
        htotx.extend([htot[0][0]])
        htoty.extend([htot[1][0]])
        htotz.extend([htot[2][0]])
        hx.extend([h[0][0]])
        hy.extend([h[1][0]])
        hz.extend([h[2][0]])
        omegasx.extend([i[0]])
        omegasy.extend([i[1]])
        omegasz.extend([i[2]])
        omegadx.extend([i[3]])
        omegady.extend([i[4]])
        omegadz.extend([i[5]])
    return [hx, hy, hz, omegasx, omegasy, omegasz,
            omegadx, omegady, omegadz, htotx, htoty, htotz, hnorm,
            h_norm_spacecraft]

def plotSpacecraft(omega_init, tstop, delta_t, kickTime):
    time = numpy.arange(0, tstop, delta_t)
    X = omega_init
    omega = goSpacecraft(X, tstop, delta_t, kickTime)
    omega[0] = numpy.array(omega[0])
    omega[1] = numpy.array(omega[1])
    omega[2] = numpy.array(omega[2])
    return omega;
    
## Obtain quaternions from angular velocity
def quatsFromAngularVelIntegrator(X, t, wx, wy, wz):
    q1, q2, q3, q4 = X[0], X[1], X[2], X[3]
    q = numpy.array([[q1, q2, q3, q4]]).T    
    ox, oy, oz = wx(t), wy(t), wz(t)
    omegacrs = numpy.array([[0., oz, -oy, ox],
                            [-oz, 0., ox, oy],
                            [oy, -ox, 0., oz],
                            [-ox, -oy, -oz, 0.]])
    qdot = .5*numpy.dot(omegacrs, q)
    
    return [qdot[0][0], qdot[1][0], qdot[2][0], qdot[3][0]]

def obtainAllQuatsfromAngVel(q0, tstop, delta_t, ws):
    a_t = numpy.arange(0, tstop, delta_t)
    asol = integrate.odeint(quatsFromAngularVelIntegrator, q0, a_t, args=ws)
    q1, q2, q3, q4 = [], [], [], []
    for i in asol:
        q1.extend([i[0]])
        q2.extend([i[1]])
        q3.extend([i[2]])
        q4.extend([i[3]])
    return [q1, q2, q3, q4]

def goBias(tstop, gyro_t, bias_init):
    time = numpy.arange(0, tstop, gyro_t)
    biasx, biasy, biasz = [bias_init[0]], [bias_init[1]], [bias_init[2]]
    for i in range(len(time)-1):
        xrand = sum(numpy.random.normal(0, GYRO_SIGMA, 100))
        yrand = sum(numpy.random.normal(0, GYRO_SIGMA, 100))
        zrand = sum(numpy.random.normal(0, GYRO_SIGMA, 100))
        biasx.extend([biasx[-1] + xrand + numpy.random.normal(0, GYRO_NOISE_SIGMA)])
        biasy.extend([biasy[-1] + yrand + numpy.random.normal(0, GYRO_NOISE_SIGMA)])
        biasz.extend([biasz[-1] + zrand + numpy.random.normal(0, GYRO_NOISE_SIGMA)])
    bias = [biasx, biasy, biasz]
    return bias

## Dynamic Propagation Equations
def getGyroMeasurement(t, gyro_noise_sigma, ws, bs):
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

### Test measurement generation, do not use
def randomRotation(axis, angle, vector):
    A = numpy.eye(3)*numpy.cos(angle) + (1.-numpy.cos(angle))*numpy.dot(axis, axis.T) + crs(axis)*numpy.sin(angle)
    return numpy.dot(A, vector)

def getMeasurement(t, meas_sigma, q1, q2, q3, q4, earthVec, moonVec, sunVec):
    quaternion = numpy.array([[float(q1(t))],
                              [float(q2(t))],
                              [float(q3(t))],
                              [float(q4(t))]])
    axis = numpy.random.rand(3,1)
    axis = axis/numpy.linalg.norm(axis)
    angle = meas_sigma*numpy.random.rand()
    vec = meas_model(quaternion, earthVec, moonVec, sunVec)
    vec[0:3][:] = randomRotation(axis, angle, vec[0:3][:])
    vec[3:6][:] = randomRotation(axis, angle, vec[3:6][:])
    vec[6:9][:] = randomRotation(axis, angle, vec[6:9][:])
    
    firstden = (numpy.sqrt(vec[0][0]**2. + vec[1][0]**2. + vec[2][0]**2.))
    secondden = (numpy.sqrt(vec[3][0]**2. + vec[4][0]**2. + vec[5][0]**2.))
    thirdden = (numpy.sqrt(vec[6][0]**2. + vec[7][0]**2. + vec[8][0]**2.))
    vec[0][0] = vec[0][0]/firstden
    vec[1][0] = vec[1][0]/firstden
    vec[2][0] = vec[2][0]/firstden
    vec[3][0] = vec[3][0]/secondden
    vec[4][0] = vec[4][0]/secondden
    vec[5][0] = vec[5][0]/secondden
    vec[6][0] = vec[6][0]/thirdden
    vec[7][0] = vec[7][0]/thirdden
    vec[8][0] = vec[8][0]/thirdden
    return vec

def generateMeasurementArray(tstop, dt, q1, q2, q3, q4, earthVec, moonVec, sunVec):
    measlist = []
    for i in numpy.arange(0, tstop, dt):
        measlist.append(getMeasurement(i, MEAS_SIGMA, q1, q2, q3, q4, earthVec, moonVec, sunVec))
    return measlist
    
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
    Phatkp, Phist[0] = P0, P0
    xhatkp, xhist[0] = x0, x0
    qhatkp, qhist[0] = q0, q0
    time = 0.
    counter = 1
    for i in measurements[1:]:
        if counter % 10==0:
            print(counter)
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
        
def plotResults(results, q1, q2, q3, q4, biasx, biasy, biasz):
    trace = []
    quat1, quat2, quat3, quat4 = [], [], [], []
    true1, true2, true3, true4 = [], [], [], []
    x1, x2, x3 = [], [], []
    b1, b2, b3 = [], [], []
    p1, p2, p3, p4, p5, p6 = [], [], [], [], [], []
    tbias1, tbias2, tbias3 = [], [], []
    truerod1, truerod2, truerod3 = [], [], []
    attitude_error = []
    for i in range(len(results[1])):
        truequaternion = numpy.array([[q1(i), q2(i), q3(i), q4(i)]]).T
        errquat = quaternionComposition(results[1][i], quaternionInv(truequaternion))
        errquat = errquat/numpy.linalg.norm(errquat)
        deltaqhat = numpy.array([[errquat[0][0], errquat[1][0], errquat[2][0]]]).T
        deltaq4 = errquat[3][0]
        attitude_error.extend([2.*numpy.arccos(deltaq4) * (180./numpy.pi)])
        rod = 1*(_f*deltaqhat)/(_a + deltaq4)
        truerod1.extend([rod[0][0]])
        truerod2.extend([rod[1][0]])
        truerod3.extend([rod[2][0]])
        p1.extend([results[0][i][0][0]])
        p2.extend([results[0][i][1][1]])
        p3.extend([results[0][i][2][2]])
        p4.extend([results[0][i][3][3]])
        p5.extend([results[0][i][4][4]])
        p6.extend([results[0][i][5][5]])
        tbias1.extend([biasx(i)])
        tbias2.extend([biasy(i)])
        tbias3.extend([biasz(i)])
    for i in results[0]:
        trace.extend([numpy.trace(i)])
    for i in results[1]:
        quat1.extend([i[0][0]])
        quat2.extend([i[1][0]])
        quat3.extend([i[2][0]])
        quat4.extend([i[3][0]])
    for i in numpy.arange(0,len(results[1]),1):
        true1.extend([q1(i)])
        true2.extend([q2(i)])
        true3.extend([q3(i)])
        true4.extend([q4(i)])
    for i in results[2]:
        x1.extend([i[0][0]])
        x2.extend([i[1][0]])
        x3.extend([i[2][0]])
        b1.extend([i[3][0]])
        b2.extend([i[4][0]])
        b3.extend([i[5][0]])

    plt.plot(quat1[90:200], 'r-', label='$q_{1}$')
    plt.plot(true1[90:200], 'r:', label='$q^{True}_{1}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(quat2[90:200], 'g-', label='$q_{2}$')
    plt.plot(true2[90:200], 'g:', label='$q^{True}_{2}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(quat3[90:200], 'b-', label='$q_{3}$')
    plt.plot(true3[90:200], 'b:', label='$q^{True}_{3}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
        
    plt.plot(quat4[90:200], 'y-', label='$q_{4}$')
    plt.plot(true4[90:200], 'y:', label='$q^{True}_{4}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Quaternions')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(b1, 'r-', label='$b_{1}$')
    plt.plot(b2, 'g-', label='$b_{2}$')
    plt.plot(b3, 'b-', label='$b_{3}$')
    plt.plot(tbias1,'r:', label='$b_{1}^{True}$')
    plt.plot(tbias2,'g:', label='$b_{2}^{True}$')
    plt.plot(tbias3,'b:', label='$b_{3}^{True}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Gyro Bias')
    plt.xlabel('Seconds')
    plt.show()
    
    plt.plot(x1, 'r-', label='$rod_{1}$')
    plt.plot(x2, 'g-', label='$rod_{2}$')
    plt.plot(x3, 'b-', label='$rod_{3}$')
    plt.plot(truerod1,'r:', label='$rod_{1}^{True}$')
    plt.plot(truerod2,'g:', label='$rod_{2}^{True}$')
    plt.plot(truerod3,'b:', label='$rod_{3}^{True}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Estimated and True Rodrigues Parameters')
    plt.xlabel('Seconds')
    plt.ylim([-0.1,0.1])
    plt.show()
    
    plt.plot(trace, label='Trace of $P_k$')
    plt.title('Trace of Covariance Matrix')
    plt.xlabel('$k$')
    plt.yscale('log')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    
    plt.plot(numpy.array(b1) - numpy.array(tbias1), 'r-', label='$b_{1}^{error}$')
    plt.plot(numpy.array(b2) - numpy.array(tbias2), 'g-', label='$b_{2}^{error}$')
    plt.plot(numpy.array(b3) - numpy.array(tbias3), 'b-', label='$b_{3}^{error}$')

    plt.title('Bias Error')
    plt.xlabel('Seconds')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    
    plt.plot(numpy.array(x1) - numpy.array(truerod1), 'r-', label='$err_{1}$')
    plt.plot(numpy.array(x2) - numpy.array(truerod2), 'g-', label='$err_{2}$')
    plt.plot(numpy.array(x3) - numpy.array(truerod3), 'b-', label='$err_{3}$')

    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('True Rodrigues Errors')
    plt.xlabel('Seconds')
    plt.ylim([-0.1,0.1])
    plt.show()
    
    plt.plot(attitude_error, label='Error (deg.)')
    plt.title('Attitude Error for UKF - Degrees')
    plt.xlabel('Seconds')
    plt.ylabel('Degrees')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.yscale('log')
    plt.show()

def runAttitudeUKFWithKick(satPos, moonPos, sunPos, cameradt, measurements, x0, quat, P0, omega_init, kickTime):

    angular_momentum_history = plotSpacecraft(omega_init, TOTAL_INTEGRATION_TIME, INTEGRATION_TIMESTEP, kickTime);
    totaltime = numpy.arange(0, TOTAL_INTEGRATION_TIME, INTEGRATION_TIMESTEP)
    hx = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[9])
    hy = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[10])
    hz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[11])
    omegax = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[3])
    omegay = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[4])
    omegaz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[5])
    # Obtain quats
    # TODO: Is this fixed to [0,0,0,1] or is it the last q0?
    q0 = quat/numpy.linalg.norm(quat)
    #     quaternion_history = obtainAllQuatsfromAngVel([0., 0., 0., 1.], TOTAL_INTEGRATION_TIME, INTEGRATION_TIMESTEP, (omegax, omegay, omegaz));
    quaternion_history = obtainAllQuatsfromAngVel(q0.flatten(), TOTAL_INTEGRATION_TIME, INTEGRATION_TIMESTEP, (omegax, omegay, omegaz));
    q1 = InterpolatedUnivariateSpline(totaltime, quaternion_history[0])
    q2 = InterpolatedUnivariateSpline(totaltime, quaternion_history[1])
    q3 = InterpolatedUnivariateSpline(totaltime, quaternion_history[2])
    q4 = InterpolatedUnivariateSpline(totaltime, quaternion_history[3])
    # Propagate bias
    bias_history = goBias(TOTAL_INTEGRATION_TIME, GYRO_SAMPLE_RATE, BIAS_INIT);
    gyrotime = numpy.arange(0, TOTAL_INTEGRATION_TIME, GYRO_SAMPLE_RATE)
    biasx = InterpolatedUnivariateSpline(gyrotime, bias_history[0])
    biasy = InterpolatedUnivariateSpline(gyrotime, bias_history[1])
    biasz = InterpolatedUnivariateSpline(gyrotime, bias_history[2])
    
    earthVec = (-1*satPos)/numpy.linalg.norm(satPos)
    moonVec = (moonPos - satPos)/numpy.linalg.norm((moonPos - satPos))
    sunVec = (sunPos - satPos)/numpy.linalg.norm(sunPos - satPos)
    
    # TODO: dummy measurements, remove this line in actual code
    measurements = generateMeasurementArray(TOTAL_INTEGRATION_TIME,cameradt, q1, q2, q3, q4, earthVec, moonVec, sunVec)
    print('Total time: {}, Measurements: {}'.format(len(totaltime), len(measurements)))
    # Run UKF
    sigmas = generateSigmas(x0, P0)
    err_quats = makeErrorQuaternion(sigmas)
    quat_pert = perturbQuaternionEstimate(err_quats,q0)
    prop_quat = propagateQuaternion(quat_pert, sigmas, 0, (omegax, omegay, omegaz), (biasx, biasy, biasz), cameradt)
    quat_errors = propagatedQuaternionError(prop_quat)
    new_sigs = recoverPropSigma(quat_errors, sigmas)
    pred_mean = predictedMean(new_sigs)
    pred_cov = predictedCov(new_sigs, pred_mean)
    hlist = hFromSigs(prop_quat, earthVec, moonVec, sunVec)
    zmean = meanMeasurement(hlist)
    Pzzmat = Pzz(hlist, zmean)
    Pxzmat = Pxz(new_sigs, pred_mean, hlist, zmean)
    K = getGain(Pxzmat, Pzzmat)
    x_new = updateXhat(pred_mean, K, measurements[0], zmean)
    P_new = updatePhat(pred_cov, K, Pzzmat)
    new_quat = updateQuaternionEstimate(x_new, prop_quat[0])

    x0 = resetSigmaSeed(x_new)
    P0 = P_new
    q0 = new_quat

    results = UKF(cameradt, P0, x0, q0, omegax, omegay, omegaz, biasx, biasy, biasz, earthVec, moonVec, sunVec, measurements)
    plotResults(results, q1, q2, q3, q4, biasx, biasy, biasz)
    
if __name__=='__main__':
    satPos = numpy.array([[-15015.40312811, -23568.9768009, 2241.5049235]]).T
    moonPos = numpy.array([[-22184.418543, -314381.535181, -97722.516592]]).T
    sunPos = numpy.array([[-16452998.593032002, 134248022.50461, -58196377.831276014]]).T
    cameradt = 1
    # TODO: Is this a constant, or a parameter
    x0 = numpy.array([[0., 0., 0., 0., 0., 0.]]).T
    quat = numpy.array([[numpy.random.randn(), numpy.random.randn(), numpy.random.randn(), numpy.random.randn()]]).T
    runAttitudeUKFWithKick(satPos, moonPos, sunPos, cameradt, None, x0, quat, P0, [0., 0.001, 2., 0., 0., 0.], 200)