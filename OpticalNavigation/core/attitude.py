import numpy
from mpl_toolkits.mplot3d import Axes3D
from numpy.linalg import inv
from numpy.linalg import pinv
from scipy.stats import norm
from scipy import integrate
from numpy import random
from copy import deepcopy
from scipy.interpolate import InterpolatedUnivariateSpline
import matplotlib.pyplot as plt

# Gyro-Based UKF for Quaternion Estimation

# We need to decide on some parameters of the problem (like inertias, etc.). Start by defining mass, height, depth, width, and inertia tensor for the Body:
m = 10   #spacecraft mass, kg
h = .3   #spacecraft height, meters
w = .3   #spacecraft width, meters
d = .1   #spacecraft depth, meters
Ib = numpy.array([[(1./12.)*m*(h**2. + d**2.), 0., 0.],
                  [0., (1./12.)*m*(w**2. + d**2.), 0.],
                  [0., 0., (1./12.)*m*(w**2. + h**2.)]]) #spacecraft inertia tensor

# Kane Damper
mdamp = 8 #damper mass in kg
mrad = 0.1 #damper radius in meters
c = 0.9    #damping coefficient
Id = numpy.array([[(2./5.)*mdamp*mrad**2., 0., 0.],
                  [0., (2./5.)*mdamp*mrad**2., 0.],
                  [0., 0., (2./5.)*mdamp*mrad**2.]])

# Initialize spacecraft quaternion and angular velocity, along with integration time, integration timestep, and standard deviation of stochastic angular accelerations:
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

# attitude dynamics

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
        h = numpy.dot(Ib, omega_spacecraft) # Spacecraft Angular Momentum (inertial frame)
        h_norm_spacecraft.extend([numpy.linalg.norm(h)])
        hdamp = numpy.dot(Id, omega_damper + omega_spacecraft) # Spacecraft Angular Momentum (inertial frame)
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

if __name__ == "__main__":        
    # Run propogation with thruster fire at t=100 seconds
    angular_momentum_history = plotSpacecraft(omega_init, tstop, delta_t);
    totaltime = numpy.arange(0, tstop, delta_t)
    hx = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[9])
    hy = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[10])
    hz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[11])
    omegax = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[3])
    omegay = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[4])
    omegaz = InterpolatedUnivariateSpline(totaltime, angular_momentum_history[5])

    quaternion_history = plotQuaternion(omega_init, tstop, delta_t, q0);
    q1 = InterpolatedUnivariateSpline(totaltime, quaternion_history[0])
    q2 = InterpolatedUnivariateSpline(totaltime, quaternion_history[1])
    q3 = InterpolatedUnivariateSpline(totaltime, quaternion_history[2])
    q4 = InterpolatedUnivariateSpline(totaltime, quaternion_history[3])

    # TODO: Total angular momentum increases after damping
