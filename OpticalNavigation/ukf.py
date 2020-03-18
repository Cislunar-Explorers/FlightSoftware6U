import numpy as np

Q = np.diag(np.array([1, 1, 1, 1e-5, 1e-6, 1e-5], dtype=np.float))

def runUKF(moonEph, sunEph, measurements, initState, dt):
    """
    One full execution of the ukf
    [moonEph]: Moon ephemeris vector (1x6)
    [sunEph]: Sun ephemeris vector (1x6)
    [measurements]: measurement vector (6x1)
    [initEstimate]: state vector from previous execution (or start state) (6x1)
    Returns:
    [stateEstimate]: (6x1) new state vector
    """
    # How wrong our dynamics model is? e.g. how off in variance will we be due
    # to solar radiation pressure, galactic particles, and bad gravity model? 
    # Units: (km^2)

def main():
    print(runUKF(None, None, None, None, 60))

if __name__ == "__main__":
    main()

