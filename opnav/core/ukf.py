from core.const import (
    CameraMeasurementVector,
    CovarianceMatrix,
    EphemerisVector,
    MainThrustInfo,
    Matrix6x6,
    TrajUKFConstants,
    TrajectoryEstimateOutput,
    TrajectoryStateVector,
)
import numpy as np
import math
from core.const import TrajUKFConstants as Const


def __length(M):
    """
    Equivalent to Matlab's len()
    """
    assert len(M.shape) >= 2
    return max(M.shape[0], M.shape[1])


def __makeSigmas(initState, Sx, Sv, nx, nv, constant):
    initState = initState.flatten()
    noise = np.zeros((6, 2 * (nx + nv) + 1))
    sigmas = np.zeros((6, 2 * (nx + nv) + 1))
    sigmas[:, 0] = initState

    # Offset sigma point positively in state (by chol(P))
    for i in range(1, nx + 1):
        sigmas[:, i] = initState + constant * Sx[:, i - 1]
        noise[:, i] = np.zeros((6,))

    # Offset sigma point negatively in state (by chol(P))
    for i in range(nx + 1, 2 * nx + 1):
        sigmas[:, i] = initState - constant * Sx[:, i - nx - 1]
        noise[:, i] = np.zeros((6,))

    # Offset sigma point positively in dynamic noise (by chol(Q))
    for i in range(2 * nx + 1, 2 * nx + nv + 1):
        sigmas[:, i] = initState
        noise[:, i] = constant * Sv[:, i - 2 * nx - 1]

    # Offset sigma point negatively in dynamic noise (by chol(Q))
    for i in range(2 * nx + nv + 1, 2 * (nx + nv) + 1):
        sigmas[:, i] = initState
        noise[:, i] = -constant * Sv[:, i - 2 * nx - nv - 1]

    return sigmas, noise


def __G(rec, rem, rcm, rcs, res, thrust):
    """
    [thrust] is 3D acceleration vector.
    """
    return (
        -Const.ue * (rec / (np.linalg.norm(rec) ** 3))
        + Const.um
        * (
            (rem - rec) / ((rcm.dot(rcm.T)) ** (3 / 2))
            - rem / (np.linalg.norm(rem) ** 3)
        )
        + Const.us
        * (
            (res - rec) / ((rcs.dot(rcs.T)) ** (3 / 2))
            - res / (np.linalg.norm(res) ** 3)
        )
        + thrust
    )


def __attitudeMatrix(quaternion):
    q1, q2, q3, q4 = quaternion
    i, j, k, r = q1, q2, q3, q4
    s = 1.0 / (np.linalg.norm(quaternion) ** 2)
    return np.array(
        [
            [
                1 - 2 * s * (j ** 2 + k ** 2),
                2 * s * (i * j - k * r),
                2 * s * (i * k + j * r),
            ],
            [
                2 * s * (i * j + k * r),
                1 - 2 * s * (i * i + k * k),
                2 * s * (j * k - i * r),
            ],
            [
                2 * s * (i * k - j * r),
                2 * s * (j * k + i * r),
                1 - 2 * s * (i * i + j * j),
            ],
        ],
        dtype=np.float64,
    )


def __dynamics_model(state, dt, moonEph, sunEph, main_thrust_info):
    """
    Runge Kutta 4th Order, one timestep
    [state]: Sigma point + noise (6x1)
    [dt]: time elapsed since last execution (seconds)
    [main_thrust_info]: dictionary containing:
        [kick_orientation]: (4x1) quaterion representing kick acceleration
                            (None) if no kick occured
        [acceleration_magnitude]: float main thrust acceleration
        [kick_duration]: float main thrust fire duration
    Ephemeris is in km, km/s
    Returns:
    Output is also in km
    """
    moonEph = moonEph * 1000
    sunEph = sunEph * 1000
    rec = state[0:3].T * 1000
    rec_dot = state[3:6].T * 1000
    rem = moonEph[0, 0:3]
    res = sunEph[0, 0:3]
    rcm = rec - rem
    rcs = rec - res
    position = rec + rec_dot * dt
    # Convert thrust vector into ECI frame
    net_acceleration_thrust = np.zeros((1, 3))
    if main_thrust_info is not None:
        kick_orientation = main_thrust_info.get_kick_orientation().data
        acc_mag = main_thrust_info.get_acceleration_magnitude()
        Aq = __attitudeMatrix(kick_orientation).reshape(3, 3)
        net_acceleration_thrust = (
            np.dot(Aq, TrajUKFConstants.ACCELERATION_DIR.data[:3].T.reshape(3, 1))
            * acc_mag
        ).reshape(1, 3)

    # Inlined RK4__calculate_cam_measurements
    n1 = __G(position, rem, rcm, rcs, res, thrust=net_acceleration_thrust)
    net_acceleration_thrust = np.zeros((1, 3))
    n2 = __G(position + dt * n1 / 2, rem, rcm, rcs, res, thrust=net_acceleration_thrust)
    n3 = __G(position + dt * n2 / 2, rem, rcm, rcs, res, thrust=net_acceleration_thrust)
    n4 = __G(position + n3 * dt, rem, rcm, rcs, res, thrust=net_acceleration_thrust)
    velocity = (
        rec_dot + (1.0 / 6.0) * (n1 + 2 * n2 + 2 * n3 + n4) * dt
    )  # TODO: main thrust fire delta time

    return np.concatenate((position, velocity), axis=None) / 1000


def __measModel(traj, moonEph, sunEph, const):
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
    dmx = moonEph[0] * 1000  # Convert km to m
    dmy = moonEph[1] * 1000
    dmz = moonEph[2] * 1000
    dsx = sunEph[0] * 1000  # Convert km to m
    dsy = sunEph[1] * 1000
    dsz = sunEph[2] * 1000
    p_c2 = x ** 2 + y ** 2 + z ** 2
    p_c = math.sqrt(p_c2)

    p_cm = math.sqrt((dmx - x) ** 2 + (dmy - y) ** 2 + (dmz - z) ** 2)
    p_cs = math.sqrt((dsx - x) ** 2 + (dsy - y) ** 2 + (dsz - z) ** 2)

    num1 = -x * (dmx - x) - y * (dmy - y) - z * (dmz - z)
    num2 = -x * (dsx - x) - y * (dsy - y) - z * (dsz - z)
    num3 = (dmx - x) * (dsx - x) + (dmy - y) * (dsy - y) + (dsz - z) * (dmz - z)
    # Pixel Separation Between Bodies
    z1 = 0  # E to M
    if num1 / (p_c * p_cm) > 1:
        z1 = 0
    elif num1 / (p_c * p_cm) < -1:
        z1 = np.pi
    else:
        z1 = math.acos(num1 / (p_c * p_cm))
    z2 = 0  # E to S
    if num2 / (p_c * p_cs) > 1:
        z2 = 0
    elif num2 / (p_c * p_cs) < -1:
        z2 = np.pi
    else:
        z2 = math.acos(num2 / (p_c * p_cs))
    z3 = 0  # M to S
    if num3 / (p_cm * p_cs) > 1:
        z3 = 0
    elif num3 / (p_cm * p_cs) < -1:
        z3 = np.pi
    else:
        z3 = math.acos(num3 / (p_cm * p_cs))

    # Pixel Diameter of Bodies
    z4 = 2 * math.atan(Const.re / p_c)  # E
    z5 = 2 * math.atan(Const.rm / p_cm)  # M
    z6 = 2 * math.atan(Const.rs / p_cs)  # S

    # Note: multiply measurement vector by "const" to convert from angles to pixels
    return np.array([z1, z2, z3, z4, z5, z6]).T * const


def __getMeans(propSigmas, sigmaMeasurements, centerWeight, otherWeight):
    """
    Weighted sums, otherwise known as x_bar or z_bar
    [propSigmas]: Propagated sigma points (6xN) where N = number of sigma points
    [sigmaMeasurements]: estimated measurements for propagated sigma points (6xN)
    Weights for main sigma point and generated sigma points correspondingly
    Returns:
    [xMean]: weighted sum of propSigmas (6x1)
    [zMean]: weighted sum of estimated measurements (6x1)
    """
    xMean = centerWeight * propSigmas[:, 0]  # 6x1
    zMean = centerWeight * sigmaMeasurements[:, 0]

    xMean = otherWeight * np.sum(propSigmas[:, 1:], 1).reshape(
        xMean.shape[0], 1
    ) + xMean.reshape(xMean.shape[0], 1)
    zMean = otherWeight * np.sum(sigmaMeasurements[:, 1:], 1).reshape(
        zMean.shape[0], 1
    ) + zMean.reshape(zMean.shape[0], 1)
    return xMean, zMean


def __findCovariances(
    xMean,
    zMean,
    propSigmas,
    sigmaMeasurements,
    centerWeight,
    otherWeight,
    alpha,
    beta,
    R,
):
    """
    Calculating weighted sums of Covariances
    [xMean,zMean,propSigmas,sigmaMeasurements, weights] See getMeans for description
    [alpha]
    [beta]
    [R]: Sensor error
    Returns:
    [Pxx, Pxz, Pzz]
    """
    centerrWeight = centerWeight + 1 - alpha ** 2 + beta
    xx = propSigmas[:, 0].reshape(6, 1) - xMean
    zz = sigmaMeasurements[:, 0].reshape(6, 1) - zMean

    Pxx = centerrWeight * (xx.dot(xx.T))
    Pxz = centerrWeight * (xx.dot(zz.T))
    Pzz = centerrWeight * (zz.dot(zz.T))

    xx2 = propSigmas[:, 1:] - xMean
    zz2 = sigmaMeasurements[:, 1:] - zMean

    for i in range(__length(xx2)):
        Pxx = otherWeight * xx2[:, i].reshape(6, 1).dot(xx2[:, i].reshape(1, 6)) + Pxx
        Pxz = otherWeight * xx2[:, i].reshape(6, 1).dot(zz2[:, i].reshape(1, 6)) + Pxz
        Pzz = otherWeight * zz2[:, i].reshape(6, 1).dot(zz2[:, i].reshape(1, 6)) + Pzz
    Pzz = Pzz + R
    return Pxx, Pxz, Pzz


def __newEstimate(
    xMean: np.ndarray,
    zMean: np.ndarray,
    Pxx: np.ndarray,
    Pxz: np.ndarray,
    Pzz: np.ndarray,
    measurements: np.ndarray,
    R: np.ndarray,
    dynamicsOnly=False,
) -> TrajectoryEstimateOutput:
    """
    Calculates new state given weighted sums of expected measurements and propogated sigmas (dynamics model)
    Returns:
    [xNew]: New state  (6x1)
    [pNew]: New covariance matrix (6x6)
    [K]: Kalman gain
    """
    # Moore-Penrose Pseudoinverse
    if not dynamicsOnly:
        K = Pxz.dot(np.linalg.pinv(Pzz))
    else:
        K = np.zeros((6, 6))  # To test dynamics Model

    measurements[-1] = zMean[-1]
    xNew = xMean + K.dot(measurements - zMean)
    pNew = Pxx - K.dot(R.dot(K.T))
    return TrajectoryEstimateOutput(
        new_state=TrajectoryStateVector.from_numpy_array(state=xNew),
        new_P=CovarianceMatrix(pNew),
        K=Matrix6x6(K),
    )


def runTrajUKF(
    moonEph: EphemerisVector,
    sunEph: EphemerisVector,
    measurements: CameraMeasurementVector,
    initState: TrajectoryStateVector,
    dt: np.float64,
    P: CovarianceMatrix,
    main_thrust_info: MainThrustInfo = None,
    dynamicsOnly: bool = False,
) -> TrajectoryEstimateOutput:
    """
    Propogates dynamics model with instantaneous impulse from the main thruster.
    Due to the processing delays of the system and the short impulse duration,
    the thrust is modeled as an instantenous acceleration on the spacecraft.
    [moonEph]: Moon ephemeris vector (1,6)
        Format: [x pos (km), y pos (km), z pos (km), vx (km/s), vy (km/s), vz (km/s)] (J2000 ECI)
    [sunEph]: Sun ephemeris vector (1,6)
        Format: [x pos (km), y pos (km), z pos (km), vx (km/s), vy (km/s), vz (km/s)] (J2000 ECI)
    [measurements]: measurement vector (6x1)
        Format: [z1, z2, z3, z4, z5, z6]
            z1 = pixel distance E-M
            z2 = pixel distance E-S
            z3 = pixel distance S-M
            z4, z5, z6 = pixel widths of E, M, S
        Can be None if running dynamics model only
    [initEstimate]: state vector from previous execution (or start state) (6x1)
        Format: [x pos (km), y pos (km), z pos (km), vx (km/s), vy (km/s), vz (km/s)] (J2000 ECI)
    [dt]: time elapsed since last execution (seconds)
    [P]: initial covariance matrix for state estimate
    [main_thrust_info]: Dictionary representing main thrust data
        (4x1 quaternion 'kick_orientation', float 'acceleration_magnitude', float 'kick_duration')
    [dynamicsOnly]: trust the dynamics model over measurements (helpful for testing)
    Returns:
    [xNew, pNew, K]: (6x1) new state vector, (6x6) new state covariance estimate, kalman gain
    """
    # measurements[3] = 0
    # measurements[4] = 0
    # measurements[5] = 0
    # print(f'moonEph: {moonEph.data}')
    # print(f'sunEph: {sunEph.data}')
    # print(f'measurements: {measurements.data}')
    # print(f'initState: {initState.data}')
    # print(f'dt: {dt}')
    # print(f'P: {P.data}')
    if measurements is None:
        measurements = CameraMeasurementVector(0, 0, 0, 0, 0, 0)
    moonEph.data = moonEph.data.reshape(1, 6)
    sunEph.data = sunEph.data.reshape(1, 6)
    measurements.data = measurements.data.reshape(6, 1)
    initState.data = initState.data.reshape(6, 1)
    P.data = P.data.reshape(6, 6)
    if main_thrust_info is not None:
        main_thrust_info.get_kick_orientation().data = (
            main_thrust_info.get_kick_orientation().data.reshape(4, 1)
        )

    nx = __length(P.data)
    nv = __length(Const.Q)
    k = 3 - nx
    lmbda = Const.alpha ** 2 * (nx + k) - nx  # tunable
    constant = np.sqrt(nx + nv + lmbda)
    centerWeight = lmbda / (nx + nv + lmbda)
    otherWeight = 1 / (2 * (nx + nv + lmbda))

    const = 3000

    # TODO: Remove test pixel to angle conversion
    # measurements /= const

    Sx = np.linalg.cholesky(P.data)

    # Generate Sigma Points
    sigmas, noise = __makeSigmas(initState.data, Sx, Const.Sv, nx, nv, constant)

    # Propogate Sigma Points
    propSigmas = np.zeros_like(sigmas)  # Initialize

    # Proprogate Sigma Points by running through dynamics model/function
    for j in range(__length(sigmas)):
        propSigmas[:, j] = __dynamics_model(
            sigmas[:, j] + noise[:, j],
            dt,
            moonEph.data,
            sunEph.data,
            main_thrust_info=main_thrust_info,
        )

    sigmaMeasurements = np.zeros((6, __length(propSigmas)))

    # Sigma measurements are the expected measurements calculated from
    # running the propagated sigma points through measurement model
    for j in range(__length(sigmas)):
        sigmaMeasurements[:, j] = __measModel(
            propSigmas[:, j] + np.random.multivariate_normal(np.zeros((6,)), Const.R).T,
            moonEph.data,
            sunEph.data,
            const,
        )

    # a priori estimates
    xMean, zMean = __getMeans(propSigmas, sigmaMeasurements, centerWeight, otherWeight)

    # Calculates the covariances as weighted sums
    Pxx, Pxz, Pzz = __findCovariances(
        xMean,
        zMean,
        propSigmas,
        sigmaMeasurements,
        centerWeight,
        otherWeight,
        Const.alpha,
        Const.beta,
        Const.R,
    )

    # a posteriori estimates
    return __newEstimate(
        xMean,
        zMean,
        Pxx,
        Pxz,
        Pzz,
        measurements.data.reshape(6, 1) * const
        + np.random.multivariate_normal(np.zeros((6)), Const.R).reshape(6, 1),
        Const.R,
        dynamicsOnly=dynamicsOnly,
    )
