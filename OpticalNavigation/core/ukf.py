import numpy as np
import math
from core.const import CameraParameters

# How wrong our dynamics model is? e.g. how off in variance will we be due
# to solar radiation pressure, galactic particles, and bad gravity model? 
# Units: (km^2)
Q = np.diag(np.array([1, 1, 1, 1e-5, 1e-6, 1e-5], dtype=np.float))
Sv = np.linalg.cholesky(Q)
# How bad are our sensors? 
# Units: (pixels^2), 
PIXEL_ERROR = 1
R = np.diag(np.array([1, 1, 1, 1, 1, 1], dtype=np.float)) * PIXEL_ERROR
alpha = 10e-4
beta = 2

ue = 3.986e14
um = 4.904e12
us = 1.327e20
re = 6378 * 1000  
rm = 1737 * 1000
rs = 695505 * 1000

def length(M):
    """
    Equivalent to Matlab's len()
    """
    assert(len(M.shape) >= 2)
    return max(M.shape[0], M.shape[1])

def makeSigmas(initState, Sx, Sv, nx, nv, constant):
    initState = initState.flatten()
    noise  = np.zeros((6, 2*(nx+nv)+1))
    sigmas = np.zeros((6, 2*(nx+nv)+1))
    sigmas[:,0] = initState

    # Offset sigma point positively in state (by chol(P))
    for i in range(1,nx+1):
        sigmas[:,i] = initState + constant*Sx[:,i-1]
        noise[:,i] = np.zeros((6,))
    
    # Offset sigma point negatively in state (by chol(P))
    for i in range(nx+1,2*nx+1):
        sigmas[:,i] = initState - constant*Sx[:,i-nx-1]
        noise[:,i] = np.zeros((6,))

    # Offset sigma point positively in dynamic noise (by chol(Q))
    for i in range(2*nx+1,2*nx+nv+1):
        sigmas[:,i] = initState
        noise[:,i] = constant*Sv[:,i-2*nx-1]
    
    # Offset sigma point negatively in dynamic noise (by chol(Q))
    for i in range(2*nx+nv+1,2*(nx+nv)+1):
        sigmas[:,i] = initState
        noise[:,i] = -constant*Sv[:,i-2*nx-nv-1]

    return sigmas, noise

def G(rec, rem, rcm, rcs, res):
    return -ue * ( rec/(np.linalg.norm(rec)**3) ) + um * ( (rem-rec)/((rcm*rcm.T)**(3/2)) - rem/(np.linalg.norm(rem)**3) ) + us * ( (res-rec)/((rcs*rcs.T)**(3/2)) - res/(np.linalg.norm(res)**3) )

def dynamics_model(state, dt, moonEph, sunEph):
    """
    Runge Kutta 4th Order, one timestep
    [state]: Sigma point + noise (6x1)
    [dt]: time elapsed since last execution (seconds)
    Ephemeris is in km, km/s 
    Returns:
    Output is also in km
    """
    moonEph = moonEph*1000
    sunEph = sunEph*1000
    rec = state[0:3].T*1000
    rec_dot = state[3:6].T*1000
    rem = moonEph[0, 0:3]
    res = sunEph[0, 0:3]
    rcm = rec-rem
    rcs = rec-res
    position = rec + rec_dot * dt
    n1 = G(position, rem, rcm, rcs, res)
    n2 = G(position+dt*n1/2, rem, rcm, rcs, res)
    n3 = G(position+dt*n2/2, rem, rcm, rcs, res)
    n4 = G(position+n3*dt, rem, rcm, rcs, res)
    velocity = rec_dot + (1.0/6.0)*(n1 + 2*n2 + 2*n3 + n4)*dt

    return np.concatenate((position, velocity), axis=None) / 1000

def measModel(traj, moonEph, sunEph, const):
    """
    Expected measurements based on propogated sigmas
    [traj]: propogated sigma point with gaussian randomness (6x1)
    Ephemiris (1,6)
    [const]: camera constant: PixelWidth/FOV (pixels/radians)
    Returns:
    [h]: Expected measurement vector [z1 z2 z3 z4 z5 z6] (6x1)
    """
    moonEph = moonEph.flatten()
    sunEph = sunEph.flatten()
    # Assumes traj is Nx6 where N = #timesteps
    # For testing, reverse the indicies (run = # on left, test = # on right)
    x = traj[0] * 1000
    y = traj[1] * 1000
    z = traj[2] * 1000
    dmx = moonEph[0] * 1000 # Convert km to m
    dmy = moonEph[1] * 1000
    dmz = moonEph[2] * 1000
    dsx = sunEph[0] * 1000 # Convert km to m
    dsy = sunEph[1] * 1000
    dsz = sunEph[2] * 1000
    p_c2 = x**2 + y**2 + z**2
    p_c = math.sqrt(p_c2)

    p_cm = math.sqrt((dmx-x)**2 + (dmy-y)**2 + (dmz-z)**2);
    p_cs = math.sqrt((dsx-x)**2 + (dsy-y)**2 + (dsz-z)**2);

    num1 = -x*dmx - y*dmy - z*dmz + p_c2;
    num2 = -x*dsx - y*dsy - z*dsz + p_c2;
    num3 = dmx*(dsx-x) + dmy*(dsy-y) + dsz*(dmz-z) - z*dmz - x*dsx - y*dsy + p_c2;

    # Pixel Separation Between Bodies 
    z1 = const * math.acos(num1/(p_c*p_cm))       # E to M
    z2 = const * math.acos(num2/(p_c*p_cs))       # E to S
    z3 = const * math.acos(num3/(p_cm*p_cs))      # M to S

    # Pixel Diameter of Bodies
    z4 = 2*const * math.atan(re/p_c)               # E
    z5 = 2*const * math.atan(rm/p_cm)              # M 
    z6 = 2*const * math.atan(rs/p_cs)              # S

    return np.array([z1, z2, z3, z4, z5, z6]).T

def getMeans(propSigmas, sigmaMeasurements, centerWeight, otherWeight):
    """
    Weighted sums, otherwise known as x_bar or z_bar
    [propSigmas]: Propagated sigma points (6xN) where N = number of sigma points
    [sigmaMeasurements]: estimated measurements for propagated sigma points (6xN)
    Weights for main sigma point and generated sigma points correspondingly
    Returns:
    [xMean]: weighted sum of propSigmas (6x1)
    [zMean]: weighted sum of estimated measurements (6x1)
    """
    xMean = centerWeight * propSigmas[:,0] # 6x1
    zMean = centerWeight * sigmaMeasurements[:,0]

    xMean = otherWeight*np.sum(propSigmas[:,1:],1).reshape(xMean.shape[0],1) + xMean.reshape(xMean.shape[0],1)
    zMean = otherWeight*np.sum(sigmaMeasurements[:,1:],1).reshape(zMean.shape[0],1) + zMean.reshape(zMean.shape[0],1)
    return xMean, zMean

def findCovariances(xMean, zMean, propSigmas, sigmaMeasurements, centerWeight, otherWeight, alpha, beta, R):
    """
    Calculating weighted sums of Covariances 
    [xMean,zMean,propSigmas,sigmaMeasurements, weights] See getMeans for description
    [alpha]
    [beta]
    [R]: Sensor error
    Returns:
    [Pxx, Pxz, Pzz]
    """
    centerWeight = centerWeight + 1 - alpha**2 + beta
    xx = propSigmas[:,0] - xMean
    zz = sigmaMeasurements[:,0] - zMean

    Pxx = centerWeight * (xx*xx.T)
    Pxz = centerWeight * (xx*zz.T)
    Pzz = centerWeight * (zz*zz.T)

    xx2 = propSigmas[:,1:] - xMean
    zz2 = sigmaMeasurements[:,1:] - zMean

    for i in range(length(xx2)):
        Pxx = otherWeight * xx2[:,i]*xx2[:,i].T + Pxx
        Pxz = otherWeight * xx2[:,i]*zz2[:,i].T + Pxz
        Pzz = otherWeight * zz2[:,i]*zz2[:,i].T + Pzz
    
    Pzz = Pzz + R
    return Pxx, Pxz, Pzz

def newEstimate(xMean, zMean, Pxx, Pxz, Pzz, measurements, R, initState, dynamicsOnly=False):
    """
    Calculates new state given weighted sums of expected measurements and propogated sigmas (dynamics model)
    Returns:
    [xNew]: New state  (6x1)
    [pNew]: New covariance matrix (6x6)
    [K]: Kalman gain
    """
    # Moore-Penrose Pseudoinverse
    if not dynamicsOnly:
        K = Pxz*np.linalg.pinv(Pzz);
    else:
        K = np.zeros((6,6)); # To test dynamics Model
    xNew = xMean + K.dot(measurements - zMean)
    pNew = Pxx - K*R*K

    return xNew, pNew, K

def runUKF(moonEph, sunEph, measurements, initState, dt, P, dynamicsOnly=False):
    """
    One full execution of the ukf
    [moonEph]: Moon ephemeris vector (1,6)
    [sunEph]: Sun ephemeris vector (1,6)
    [measurements]: measurement vector (6x1)
    [initEstimate]: state vector from previous execution (or start state) (6x1)
    [dt]: time elapsed since last execution (seconds)
    [P]: initial covariance matrix for state estimate
    [dynamicsOnly]: trust the dynamics model over measurements (helpful for testing)
    Returns:
    [xNew, pNew, K]: (6x1) new state vector, (6x6) new state covariance estimate, kalman gain
    """

    nx = length(P)
    nv = length(Q)
    k = 3-nx;
    lmbda = alpha**2*(nx+k)-nx # tunable
    constant = np.sqrt(nx+nv+lmbda)
    centerWeight = lmbda/(nx+nv+lmbda)
    otherWeight = 1/(2*(nx+nv+lmbda))

    const = CameraParameters.hPix/(CameraParameters.hFov*np.pi/180) # camera constant: PixelWidth/FOV  [pixels/radians]

    Sx = np.linalg.cholesky(P)
    
    # Generate Sigma Points
    sigmas, noise = makeSigmas(initState, Sx, Sv, nx, nv, constant)

    # Propogate Sigma Points
    propSigmas = np.zeros_like(sigmas) # Initialize

    # Proprogate Sigma Points by running through dynamics model/function
    for j in range(length(sigmas)):
        propSigmas[:,j] = dynamics_model(sigmas[:,j]+noise[:,j], dt, moonEph, sunEph);
    
    sigmaMeasurements = np.zeros((6,length(propSigmas)))
    
    # Sigma measurements are the expected measurements calculated from
    # running the propagated sigma points through measurement model
    for j in range(length(sigmas)):
        sigmaMeasurements[:,j] = measModel(propSigmas[:,j] + np.random.multivariate_normal(np.zeros((6,)),R).T, moonEph, sunEph, const) 
    
    # a priori estimates
    xMean, zMean = getMeans(propSigmas, sigmaMeasurements, centerWeight, otherWeight)

    # Calculates the covariances as weighted sums
    Pxx, Pxz, Pzz = findCovariances(xMean, zMean, propSigmas, sigmaMeasurements, centerWeight, otherWeight, alpha, beta, R)

    # a posteriori estimates
    return newEstimate(xMean, zMean, Pxx, Pxz, Pzz, measurements + np.random.multivariate_normal(np.zeros((6)),R).reshape(measurements.shape[0],1), R, initState, dynamicsOnly=dynamicsOnly)


def main():
    traj = (np.array([883.9567, 1.023e+03, 909.665, 65.648, 11.315, 28.420], dtype=np.float)).reshape(6,1)
    moonEph = (np.array([1.536e+05, -3.723e+05, 2.888e+03, 0.9089, 0.3486, -0.0880], dtype=np.float)).reshape(1,6)
    sunEph = (np.array([-3.067e+07, -1.441e+08, 6.67e+03, 29.6329, -6.0859, -8.8015e-04], dtype=np.float)).reshape(1,6)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    measurements = (np.array([3783.89178515,  854.57125906, 3446.64998585,  544.40002441, 1949.59997559, 40.0], dtype=np.float)).reshape(6,1)

    xNew, pNew, K = runUKF(moonEph, sunEph, measurements, traj, 60, P)
    print(xNew)

if __name__ == "__main__":
    main()

