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

m = 10   #spacecraft mass, kg
h = .3   #spacecraft height, meters
w = .3   #spacecraft width, meters
d = .1   #spacecraft depth, meters
Ib = numpy.array([[(1./12.)*m*(h**2. + d**2.), 0., 0.],
                  [0., (1./12.)*m*(w**2. + d**2.), 0.],
                  [0., 0., (1./12.)*m*(w**2. + h**2.)]]) #spacecraft inertia tensor

mdamp = 8 #damper mass in kg
mrad = 0.1 #damper radius in meters
c = 0.9    #damping coefficient
Id = numpy.array([[(2./5.)*mdamp*mrad**2., 0., 0.],
                  [0., (2./5.)*mdamp*mrad**2., 0.],
                  [0., 0., (2./5.)*mdamp*mrad**2.]])


q0 = [0., 0., 0., 1.]                   # initial quaternion
omega_init = [0., 0.001, 2., 0., 0., 0.]# initial angular velocity
tstop = 700                             # total integration time
delta_t = 0.001                          # integration timestep
gyro_t = 0.04
gyrotime = numpy.arange(0, tstop, gyro_t)
sigma_acc = 0.                          # stochastic accelerations
gyro_sigma = 1.e-10
gyro_noise_sigma = 1.e-7
bias_init=[0., 0., 0.]
meas_sigma = 8.7e-4

def crs(vector):
    first = vector[0][0]
    second = vector[1][0]
    third = vector[2][0]
    return numpy.array([[0., -third, second],
                        [third, 0., -first],
                        [-second, first, 0.]])

def propagateSpacecraft(X, t):
    omegasc1, omegasc2, omegasc3 = X[0], X[1], X[2]
    omegad1, omegad2, omegad3 = X[3], X[4], X[5]
    omega_sc = numpy.array([[omegasc1, omegasc2, omegasc3]]).T
    omega_d  = numpy.array([[omegad1, omegad2, omegad3]]).T
    
    inner_sc = (numpy.dot(crs(omega_sc), numpy.dot(Ib, omega_sc)) - c*omega_d +
                    int(0.5*(numpy.sign(t - 100)+1))*numpy.array([[.1, 0., 0.]]).T -
                    int(0.5*(numpy.sign(t - 102)+1))*numpy.array([[.1, 0., 0.]]).T)
    omega_sc_dot = (numpy.dot(-1.*pinv(Ib), inner_sc))
    
    right_d = numpy.dot(crs(omega_sc), numpy.dot(Id, omega_d + omega_sc)) + c*omega_d
    inner_d = numpy.dot(Id, omega_sc_dot) + right_d
    omega_d_dot = numpy.dot(-1.*pinv(Id), inner_d)
    
    derivs = [omega_sc_dot[0][0], omega_sc_dot[1][0], omega_sc_dot[2][0],
              omega_d_dot[0][0], omega_d_dot[1][0], omega_d_dot[2][0]]
    
    return derivs

def goSpacecraft(X, tstop, delta_t):
    a_t = numpy.arange(0, tstop, delta_t)
    asol = integrate.odeint(propagateSpacecraft, X, a_t)
    hx, hy, hz = [], [], []
    htotx, htoty, htotz = [], [], []
    omegasx, omegasy, omegasz = [], [], []
    omegadx, omegady, omegadz = [], [], []
    h_norm_spacecraft = []
    hnorm = []
    for i in asol:
        omega_spacecraft = numpy.array([[i[0], i[1], i[2]]]).T
        omega_damper = numpy.array([[i[3], i[4], i[5]]]).T
        h = numpy.dot(Ib, omega_spacecraft)
        h_norm_spacecraft.extend([numpy.linalg.norm(h)])
        hdamp = numpy.dot(Id, omega_damper + omega_spacecraft)
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

def plotSpacecraft(omega_init, tstop, delta_t):
    time = numpy.arange(0, tstop, delta_t)
    X = omega_init
    omega = goSpacecraft(X, tstop, delta_t)
    omega[0] = numpy.array(omega[0])
    omega[1] = numpy.array(omega[1])
    omega[2] = numpy.array(omega[2])
    plt.plot(time, omega[0], label='$h_{x}^{B/N}$')
    plt.plot(time, omega[1], label='$h_{y}^{B/N}$')
    plt.plot(time, omega[2], label='$h_{z}^{B/N}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Spacecraft Angular Momentum')
    plt.xlabel('Seconds')
    plt.ylabel('Angular Momentum (SI Units)')
    plt.show()
    plt.plot(time, omega[3], label='$\omega_{x}^{B/N}$')
    plt.plot(time, omega[4], label='$\omega_{y}^{B/N}$')
    plt.plot(time, omega[5], label='$\omega_{z}^{B/N}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Spacecraft Angular Velocity')
    plt.xlabel('Seconds')
    plt.ylabel('Rad/sec')
    plt.show()
    plt.plot(time, omega[6], label='$\omega_{x}^{D/B}$')
    plt.plot(time, omega[7], label='$\omega_{y}^{D/B}$')
    plt.plot(time, omega[8], label='$\omega_{z}^{D/B}$')
    plt.title('Damper Angular Velocity (Relative to Spacecraft)')
    plt.xlabel('Seconds')
    plt.ylabel('Rad/Sec')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    plt.plot(time, omega[9], label='$h^{tot}_{x}$')
    plt.plot(time, omega[10], label='$h^{tot}_{y}$')
    plt.plot(time, omega[11], label='$h^{tot}_{z}$')
    plt.title('Total Angular Momentum')
    plt.xlabel('Seconds')
    plt.ylabel('Angular Momentum')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    plt.plot(time, omega[12], label='$||h^{tot}||$')
    plt.title('Total Angular Momentum Norm')
    plt.xlabel('Seconds')
    plt.ylabel('Angular Momentum')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
#     plt.ylim([0.16,0.19])
    plt.show()
    
    return omega;

angular_momentum_history = plotSpacecraft(omega_init, tstop, delta_t);
totaltime = numpy.arange(0, tstop, delta_t)
hx = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[9])
hy = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[10])
hz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[11])
omegax = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[3])
omegay = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[4])
omegaz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[5])

def propagateQuaternion(X, t):
    q1, q2, q3, q4 = X[0], X[1], X[2], X[3]
    q = numpy.array([[q1, q2, q3, q4]]).T    
    ox, oy, oz = omegax(t), omegay(t), omegaz(t)
    omegacrs = numpy.array([[0., oz, -oy, ox],
                            [-oz, 0., ox, oy],
                            [oy, -ox, 0., oz],
                            [-ox, -oy, -oz, 0.]])
    qdot = .5*numpy.dot(omegacrs, q)
    
    return [qdot[0][0], qdot[1][0], qdot[2][0], qdot[3][0]]

def goQuaternion(X, tstop, delta_t):
    a_t = numpy.arange(0, tstop, delta_t)
    asol = integrate.odeint(propagateQuaternion, X, a_t)
    q1, q2, q3, q4 = [], [], [], []
    for i in asol:
        q1.extend([i[0]])
        q2.extend([i[1]])
        q3.extend([i[2]])
        q4.extend([i[3]])
    return [q1, q2, q3, q4]

def plotQuaternion(omega_init, tstop, delta_t, q0):
    time = numpy.arange(0, tstop, delta_t)
    X = q0
    quat = goQuaternion(X, tstop, delta_t)
    norm = numpy.sqrt(numpy.array(quat[0])**2. +
                      numpy.array(quat[1])**2. +
                      numpy.array(quat[2])**2. +
                      numpy.array(quat[3])**2.)
    plt.plot(time, norm, label='$|q|$')
    plt.plot(time, quat[0], label='$q_{1}$')
    plt.plot(time, quat[1], label='$q_{2}$')
    plt.plot(time, quat[2], label='$q_{3}$')
    plt.plot(time, quat[3], label='$q_{4}$', alpha=0.1)
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Spacecraft Quaternion')
    plt.xlabel('Seconds')
    plt.ylim([-1.5,1.5])
    plt.show()
    return quat;

quaternion_history = plotQuaternion(omega_init, tstop, delta_t, q0);
q1 = InterpolatedUnivariateSpline(totaltime, quaternion_history[0])
q2 = InterpolatedUnivariateSpline(totaltime, quaternion_history[1])
q3 = InterpolatedUnivariateSpline(totaltime, quaternion_history[2])
q4 = InterpolatedUnivariateSpline(totaltime, quaternion_history[3])

a=1
f=2.*(a+1)

def propagateBias(X, t):
    bx, by, bz = X[0], X[1], X[2]
    return [numpy.random.normal(0, gyro_sigma),
            numpy.random.normal(0, gyro_sigma),
            numpy.random.normal(0, gyro_sigma)]

def goBias(X, tstop, gyro_t):
    a_t = numpy.arange(0, tstop, delta_t)
    asol = integrate.odeint(propagateBias, X, a_t)
    biasx, biasy, biasz = [], [], []
    for i in asol:
        biasx.extend([i[0]])
        biasy.extend([i[1]])
        biasz.extend([i[2]])
    return [biasx, biasy, biasz]

def plotBias(tstop, gyro_t, bias_init):
    time = numpy.arange(0, tstop, gyro_t)
    biasx, biasy, biasz = [bias_init[0]], [bias_init[1]], [bias_init[2]]
    for i in range(len(time)-1):
        xrand = sum(numpy.random.normal(0, gyro_sigma, 100))
        yrand = sum(numpy.random.normal(0, gyro_sigma, 100))
        zrand = sum(numpy.random.normal(0, gyro_sigma, 100))
        biasx.extend([biasx[-1] + xrand + numpy.random.normal(0, gyro_noise_sigma)])
        biasy.extend([biasy[-1] + yrand + numpy.random.normal(0, gyro_noise_sigma)])
        biasz.extend([biasz[-1] + zrand + numpy.random.normal(0, gyro_noise_sigma)])
    bias = [biasx, biasy, biasz]
    plt.plot(time, bias[0], label='$bias_{x}$')
    plt.plot(time, bias[1], label='$bias_{y}$')
    plt.plot(time, bias[2], label='$bias_{z}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Gyro Bias')
    plt.xlabel('Seconds')
    plt.show()
    return bias

bias_history = plotBias(tstop, gyro_t, bias_init);
biasx = InterpolatedUnivariateSpline(gyrotime, bias_history[0])
biasy = InterpolatedUnivariateSpline(gyrotime, bias_history[1])
biasz = InterpolatedUnivariateSpline(gyrotime, bias_history[2])

def getGyroMeasurement(t, gyro_noise_sigma):
    return (numpy.array([[float(omegax(t)), float(omegay(t)),
                          float(omegaz(t))]]).T + numpy.array([[biasx(t)],
                                                               [biasy(t)],
                                                               [biasz(t)]]) +
            numpy.random.randn(3,1)*gyro_noise_sigma)

def updateOmega(t, biasEst):
    return getGyroMeasurement(t, gyro_sigma) - biasEst

def updateBeta(betakp):
    return betakp

def computePsi(t, biasEst, sample_rate):
    omegaplus = updateOmega(t, biasEst)
    return (numpy.sin(0.5*sample_rate*numpy.linalg.norm(omegaplus))/
            numpy.linalg.norm(omegaplus)) * omegaplus

def crs(vector):
    first = vector[0][0]
    second = vector[1][0]
    third = vector[2][0]
    return numpy.array([[0., -third, second],
                        [third, 0., -first],
                        [-second, first, 0.]])

def computeBigOmega(t, biasEst, sample_rate):
    psi = computePsi(t, biasEst, sample_rate)
    oneone = numpy.cos(0.5*sample_rate*numpy.linalg.norm(updateOmega(t, biasEst)))
    psicrs = crs(psi)
    lowerright = numpy.array([[numpy.cos(0.5*sample_rate*
                                         numpy.linalg.norm(updateOmega(t, biasEst)))]])
    upperleft = oneone*numpy.eye(3) - psicrs
    upperright = psi
    lowerleft = -1*psi.T
    top = numpy.concatenate((upperleft, upperright), axis=1)
    bottom = numpy.concatenate((lowerleft, lowerright), axis=1)
    return numpy.concatenate((top, bottom), axis=0)

def updateQuaternion(t, biasEst, sample_rate, qkp):
    bigO = computeBigOmega(t, biasEst, sample_rate)
    return numpy.dot(bigO, qkp)

def test():
    qu1, qu2 = numpy.zeros(len(gyrotime)), numpy.zeros(len(gyrotime))
    qu3, qu4 = numpy.zeros(len(gyrotime)), numpy.zeros(len(gyrotime))
    gyrox, gyroy, gyroz = numpy.zeros(len(gyrotime)), numpy.zeros(len(gyrotime)), numpy.zeros(len(gyrotime))
    quat = numpy.array([[q1(0), q2(0), q3(0), q4(0)]]).T
    qu1[0] = q1(0)
    qu2[0] = q2(0)
    qu3[0] = q3(0)
    qu4[0] = q4(0)
    counter = 1
    for i in gyrotime[0:-1]:
        if i%10 == 0:
            print(i)
        newq = updateQuaternion(i, numpy.array([[biasx(i), biasy(i), biasz(i)]]).T, gyro_t, quat)
        qu1[counter] = newq[0][0]
        qu2[counter] = newq[1][0]
        qu3[counter] = newq[2][0]
        qu4[counter] = newq[3][0]
        omega = getGyroMeasurement(i, gyro_noise_sigma)
        gyrox[counter] = omega[0][0]
        gyroy[counter] = omega[1][0]
        gyroz[counter] = omega[2][0]
        counter += 1
        quat = newq
    norm = numpy.sqrt(numpy.array(qu1)**2. +
                      numpy.array(qu2)**2. +
                      numpy.array(qu3)**2. +
                      numpy.array(qu4)**2.)
#     plt.plot(gyrotime, norm, label='$|q|$')
    plt.plot(gyrotime, numpy.array(qu1) - q1(gyrotime), label='$q_{1}$')
    plt.plot(gyrotime, numpy.array(qu2) - q2(gyrotime), label='$q_{2}$')
    plt.plot(gyrotime, numpy.array(qu3) - q3(gyrotime), label='$q_{3}$')
    plt.plot(gyrotime, numpy.array(qu4) - q4(gyrotime), label='$q_{4}$')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.title('Propagated and True Quaternion Errors')
    plt.xlabel('Seconds')
#     plt.ylim([-1.5,1.5])
    plt.show()
    return qu1, qu2, qu3, qu4, gyrox, gyroy, gyroz

qu1, qu2, qu3, qu4, gyrox, gyroy, gyroz = test()
# print 'Uncomment to run.'

satPos = numpy.array([[-15015.40312811, -23568.9768009, 2241.5049235]]).T
moonPos = numpy.array([[-22184.418543, -314381.535181, -97722.516592]]).T
sunPos = numpy.array([[-16452998.593032002, 134248022.50461, -58196377.831276014]]).T

earthVec = (-1*satPos)/numpy.linalg.norm(satPos)
moonVec = (moonPos - satPos)/numpy.linalg.norm((moonPos - satPos))
sunVec = (sunPos - satPos)/numpy.linalg.norm(sunPos - satPos)

def formA(q):
    q1, q2, q3, q4 = q[0][0], q[1][0], q[2][0], q[3][0]
    return numpy.array([[1. - 2.*q2**2. - 2.*q3**2., 2.*q1*q2 - 2.*q3*q4, 2.*q1*q3 + 2.*q2*q4],
                        [2.*q1*q2 + 2.*q3*q4, 1.-2.*q1**2. - 2.*q3**2., 2.*q2*q3 - 2.*q1*q4],
                        [2.*q1*q3 - 2.*q2*q4, 2.*q1*q4 + 2.*q2*q3, 1.-2.*q1**2. - 2.*q2**2.]])

def h(state):
    A = formA(state)
    zearth = numpy.dot(A, earthVec)
    zmoon = numpy.dot(A, moonVec)
    zsun = numpy.dot(A, sunVec)
    zearth = numpy.concatenate((zearth, zmoon), axis=0)
    zearth = numpy.concatenate((zearth, zsun), axis=0)
    return zearth

def randomRotation(axis, angle, vector):
    A = numpy.eye(3)*numpy.cos(angle) + (1.-numpy.cos(angle))*numpy.dot(axis, axis.T) + crs(axis)*numpy.sin(angle)
    return numpy.dot(A, vector)

def getMeasurement(t, meas_sigma):
    quaternion = numpy.array([[float(q1(t))],
                              [float(q2(t))],
                              [float(q3(t))],
                              [float(q4(t))]])
    axis = numpy.random.rand(3,1)
    axis = axis/numpy.linalg.norm(axis)
    angle = meas_sigma*numpy.random.rand()
    vec = h(quaternion)
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

def generateMeasurementArray(tstop, dt):
    measlist = []
    for i in numpy.arange(0, tstop, dt):
        measlist.append(getMeasurement(i, meas_sigma))
    return measlist

cameradt = 1.
measurements = generateMeasurementArray(tstop,cameradt)

def showMeas():
    ex, ey, ez, em = [], [], [], []
    my, mx, mz, mm = [], [], [], []
    sx, sy, sz, sm = [], [], [], []
    etx, ety, etz = [], [], []
    mtx, mty, mtz = [], [], []
    stx, sty, stz = [], [], []
    start = 98
    stop = 120
    counter = deepcopy(start)
    for i in measurements[start:stop]:
        em.extend([numpy.linalg.norm(i[0:3])])
        mm.extend([numpy.linalg.norm(i[3:6])])
        sm.extend([numpy.linalg.norm(i[6:9])])
        ex.extend([i[0][0]])
        ey.extend([i[1][0]])
        ez.extend([i[2][0]])
        mx.extend([i[3][0]])
        my.extend([i[4][0]])
        mz.extend([i[5][0]])
        sx.extend([i[6][0]])
        sy.extend([i[7][0]])
        sz.extend([i[8][0]])
        true = h(numpy.array([[q1(counter), q2(counter), q3(counter), q4(counter)]]).T)
        etx.extend([true[0][0]])
        ety.extend([true[1][0]])
        etz.extend([true[2][0]])
        mtx.extend([true[3][0]])
        mty.extend([true[4][0]])
        mtz.extend([true[5][0]])
        stx.extend([true[6][0]])
        sty.extend([true[7][0]])
        stz.extend([true[8][0]])
        counter +=1
    plt.plot(em, label='$|\hat{e}|$')
    plt.plot(ex, label='$\hat{x}$')
    plt.plot(ey, label='$\hat{y}$')
    plt.plot(ez, label='$\hat{z}$')
    plt.plot(etx, label='$\hat{x}_{true}$')
    plt.plot(ety, label='$\hat{y}_{true}$')
    plt.plot(etz, label='$\hat{z}_{true}$')
    plt.title('Earth Unit Vector Measurements')
    plt.xlabel('Seconds')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.ylim([-1.5,1.5])
    plt.show()
    plt.plot(mm, label='$|\hat{m}|$')
    plt.plot(mx, label='$\hat{x}$')
    plt.plot(my, label='$\hat{y}$')
    plt.plot(mz, label='$\hat{z}$')
    plt.plot(mtx, label='$\hat{x}_{true}$')
    plt.plot(mty, label='$\hat{y}_{true}$')
    plt.plot(mtz, label='$\hat{z}_{true}$')
    plt.title('Moon Unit Vector Measurements')
    plt.xlabel('Seconds')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.ylim([-1.5,1.5])
    plt.show()
    plt.plot(sm, label='$|\hat{s}|$')
    plt.plot(sx, label='$\hat{x}$')
    plt.plot(sy, label='$\hat{y}$')
    plt.plot(sz, label='$\hat{z}$')
    plt.plot(stx, label='$\hat{x}_{true}$')
    plt.plot(sty, label='$\hat{y}_{true}$')
    plt.plot(stz, label='$\hat{z}_{true}$')
    plt.title('Sun Unit Vector Measurements')
    plt.xlabel('Seconds')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.ylim([-1.5,1.5])
    plt.show()
showMeas()

# Tuning Parameters for Sigma Points
Nx = 6.                                 # number of states
alpha = 0                           # determines spread of sigma points
beta = 2.                               # optimal for Gaussian distribution
kappa = -3.                             # chosen such that kappa+Nx=3
lam = 0#alpha**2. * (kappa + Nx) - Nx     # depends on other variables

P0 = numpy.array([[1.e-1, 0., 0., 0., 0., 0.],
                  [0., 1.e-1, 0., 0., 0., 0.],
                  [0., 0., 1.e-1, 0., 0., 0.],
                  [0., 0., 0., 9.7e-10, 0., 0.],
                  [0., 0., 0., 0., 9.7e-10, 0.],
                  [0., 0., 0., 0., 0., 9.7e-10]]) * 10.

Q = numpy.array([[gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_t**2., 0., 0., 0., 0., 0.],
                 [0., gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_t**2., 0., 0., 0., 0.],
                 [0., 0., gyro_noise_sigma**2. - (1./6.)*gyro_sigma**2.*gyro_t**2., 0., 0., 0.],
                 [0., 0., 0., gyro_sigma**2., 0., 0.],
                 [0., 0., 0., 0., gyro_sigma**2., 0.],
                 [0., 0., 0., 0., 0., gyro_sigma**2.]]) * .5*gyro_t

R = numpy.eye(9) * meas_sigma**2.

x0 = numpy.array([[0., 0., 0., 0., 0., 0.]]).T

# q0 = numpy.array([[ 0.01030764,  0.01030764,  0.01030764,  0.99984062]]).T
# q0 = numpy.array([[0., 0., 0., 1.]]).T
quat = numpy.array([[numpy.random.randn(), numpy.random.randn(), numpy.random.randn(), numpy.random.randn()]]).T
q0 = quat/numpy.linalg.norm(quat)

def getCholesky(Pmat, Qmat):
    return numpy.linalg.cholesky(Pmat + Q)

def extractColumns(mat):
    cols = numpy.zeros((len(mat), int(Nx), 1))
    for i in range(len(mat)):
        cols[i] = (numpy.array([mat[:,i]]).T)
    return cols

def generateSigmas(xhatkp, Phatkp):
    sigpoints = numpy.zeros((int(2*Nx + 1), int(Nx), 1))
    sigpoints[0] = numpy.array([[0., 0., 0., xhatkp[3][0], xhatkp[4][0], xhatkp[5][0]]]).T
    S = getCholesky(Phatkp, Q)
    si = extractColumns(S)
    counter = 1
    for i in si:
        sigpoints[counter] = (sigpoints[0] + numpy.sqrt(Nx+lam)*i)
        sigpoints[counter+1] = (sigpoints[0] - numpy.sqrt(Nx+lam)*i)
        counter+=2
    return sigpoints

def makeErrorQuaternion(sigmas):
    errorquat = numpy.zeros((len(sigmas), 4, 1))
    counter = 0
    for i in sigmas:
        deltap = numpy.array([[i[0][0], i[1][0], i[2][0]]]).T
        deltaq4p = ((-a*numpy.linalg.norm(deltap)**2. + 
                     f*numpy.sqrt(f**2. + (1-a**2.)*numpy.linalg.norm(deltap)**2.))/
                    (f**2. + numpy.linalg.norm(deltap)**2.))
        deltaqhatkp = (1./f)*(a + deltaq4p)*deltap
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

def propagateQuaternion(perturbed_quat_list, sigmas, t):
    time = numpy.arange(t, t+cameradt, gyro_t)
    updated_quats = numpy.zeros((len(perturbed_quat_list), 4, 1))
    for i in range(len(perturbed_quat_list)):
        bias = numpy.array([[sigmas[i][3][0], sigmas[i][4][0], sigmas[i][5][0]]]).T
        quat = perturbed_quat_list[i]
        for j in time:
            newq = updateQuaternion(j, bias, gyro_t, quat)
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
    newsigs = numpy.zeros((len(quat_errors), int(Nx), 1))
    for i in range(len(quat_errors)):
        deltaqhat = numpy.array([[quat_errors[i][0][0]],
                                 [quat_errors[i][1][0]],
                                 [quat_errors[i][2][0]]])
        deltaq4 = quat_errors[i][3][0]
        top = (f*deltaqhat)/(a + deltaq4)
        bottom = numpy.array([[old_sigmas[i][3][0]],
                              [old_sigmas[i][4][0]],
                              [old_sigmas[i][5][0]]])
        sig = numpy.concatenate((top, bottom), axis=0)
        newsigs[i]=(sig)
    return newsigs

def predictedMean(newsigs):
    xmean = (1./(Nx + lam))*(lam*newsigs[0])
    for i in newsigs[1:]:
        xmean += (1./(2.*(Nx+lam)))*i
    return xmean

def predictedCov(newsigs, pred_mean):
    P = ((lam/(Nx + lam)))*numpy.dot((newsigs[0] - pred_mean),(newsigs[0] - pred_mean).T)
    for i in newsigs[1:]:
        P += (1./(2*(Nx+lam)))*numpy.dot((i - pred_mean), (i - pred_mean).T)
    return P + Q

def hFromSigs(prop_quaternions):
    hlist = numpy.zeros((len(prop_quaternions), 9, 1))
    counter = 0
    for i in prop_quaternions:
        hlist[counter] = (h(i))
        counter +=1
    return hlist

def meanMeasurement(hlist):
    zmean = (1./(Nx + lam))*(lam * hlist[0])
    for i in hlist[1:]:
        zmean += (1./(2*(Nx+lam)))*i
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
    Pzz = ((lam/(Nx+lam)))*numpy.dot(hlist[0] - zmean, (hlist[0] - zmean).T)
    for i in hlist[1:]:
        Pzz += (1./(2*(Nx+lam)))*numpy.dot(i-zmean, (i-zmean).T)
    return Pzz + R


def Pxz(siglist, xmean, hlist, zmean):
    Pxzmat = ((lam/(Nx+lam)))*numpy.dot(siglist[0] - xmean, (hlist[0]-zmean).T)
    for i in range(1,len(siglist)):
        Pxzmat += (1./(2*(Nx+lam)))*numpy.dot(siglist[i]-xmean,(hlist[i]-zmean).T)
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
    deltaq4p = ((-a*numpy.linalg.norm(deltap)**2. + 
                 f*numpy.sqrt(f**2. + (1-a**2.)*numpy.linalg.norm(deltap)**2.))/
                (f**2. + numpy.linalg.norm(deltap)**2.))
    deltaqhatkp = (1./f)*(a + deltaq4p)*deltap
    deltaq = numpy.concatenate((deltaqhatkp, numpy.array([[deltaq4p]])), axis=0)
    new = quaternionComposition(deltaq, qhatkm)
    return new/numpy.linalg.norm(new)

def resetSigmaSeed(xhatkp):
    return numpy.array([[0., 0., 0., xhatkp[3][0], xhatkp[4][0], xhatkp[5][0]]]).T

def UKF():
    Phist = numpy.zeros((int(tstop/cameradt), 6, 6))
    qhist = numpy.zeros((int(tstop/cameradt), 4, 1))
    xhist = numpy.zeros((int(tstop/cameradt), 6, 1))
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
        prop_quats = propagateQuaternion(pert_quats, sigma_points, time)
        qhatkp1m = prop_quats[0]
        prop_err = propagatedQuaternionError(prop_quats)
        prop_sigmas = recoverPropSigma(prop_err, sigma_points)
        x_mean = predictedMean(prop_sigmas)
        p_mean = predictedCov(prop_sigmas, x_mean)
        h_list = hFromSigs(prop_quats)
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

results = UKF()

def plotResults(results):
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
        rod = 1*(f*deltaqhat)/(a + deltaq4)
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
#     plt.plot(3*numpy.sqrt(numpy.array(p4)), 'r:', label='$\pm 3\sigma_{4}$')
#     plt.plot(3*numpy.sqrt(numpy.array(p5)), 'g:', label='$\pm 3\sigma_{5}$')
#     plt.plot(3*numpy.sqrt(numpy.array(p6)), 'b:', label='$\pm 3\sigma_{6}$')
#     plt.plot(-3*numpy.sqrt(numpy.array(p4)), 'r:')
#     plt.plot(-3*numpy.sqrt(numpy.array(p5)), 'g:')
#     plt.plot(-3*numpy.sqrt(numpy.array(p6)), 'b:')
    plt.title('Bias Error')
    plt.xlabel('Seconds')
    plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
    plt.show()
    
    plt.plot(numpy.array(x1) - numpy.array(truerod1), 'r-', label='$err_{1}$')
    plt.plot(numpy.array(x2) - numpy.array(truerod2), 'g-', label='$err_{2}$')
    plt.plot(numpy.array(x3) - numpy.array(truerod3), 'b-', label='$err_{3}$')
#     plt.plot(3*numpy.sqrt(numpy.array(p1)), 'r:', label='$\pm 1\sigma_{1}$')
#     plt.plot(3*numpy.sqrt(numpy.array(p2)), 'g:', label='$\pm 1\sigma_{2}$')
#     plt.plot(3*numpy.sqrt(numpy.array(p3)), 'b:', label='$\pm 1\sigma_{3}$')
#     plt.plot(-3*numpy.sqrt(numpy.array(p1)), 'r:')
#     plt.plot(-3*numpy.sqrt(numpy.array(p2)), 'g:')
#     plt.plot(-3*numpy.sqrt(numpy.array(p3)), 'b:')
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

plotResults(results)
