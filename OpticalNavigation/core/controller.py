from core.acquisition import startAcquisition, readOmega
from core.cam_meas import cameraMeasurements
from core.ukf import runUKF
import numpy as np
import traceback

def run(currentTime, moonEph, sunEph, initState, P, dir=None):
    """
    Executes OpNav pipeline once
    [currentTime]: index into ephemiris table
    [dir]: Directory of acquired images. Should have subfolders Camera1/, Camera2/, Camera3/
    Ephemiris
    Returns:
    [xNew, P, K]: OpNav finished successfully
    [None, None, None]: Measurements did not find all three bodies
    Throws:
    Exception if measurements failed
    """
    print("Starting OpNav")
    #startAcquisition(dir)
    try:
        meas = cameraMeasurements([0, 0, readOmega()], 0.91, dir)
        if meas is None:
            print("[Opnav controller]: did not find all three bodies. Skipping...")
            return None, None, None
        else:
            xNew, pNew, K = runUKF(moonEph[currentTime].reshape(1,6), sunEph[currentTime].reshape(1,6), meas.reshape(6,1), initState, 60, P) # TODO: UKF computation errors?
            # TODO: Deposit position and velocity estimates into global location
            print("[Opnav controller]: Finished successfully {}, Gain {}".format(xNew, K))
            return xNew, pNew, K
    except Exception as e:
        traceback.print_stack()
        raise Exception("[Opnav controller]: Error while trying to compute measurements: {}".format(e))
        

def main():
    traj = (np.array([883.9567, 1.023e+03, 909.665, 65.648, 11.315, 28.420], dtype=np.float)).reshape(6,1)
    moonEph = (np.array([[1.536e+05, -3.723e+05, 2.888e+03, 0.9089, 0.3486, -0.0880],[0,0,0,0,0,0]], dtype=np.float)).reshape(2,6)
    sunEph = (np.array([[-3.067e+07, -1.441e+08, 6.67e+03, 29.6329, -6.0859, -8.8015e-04],[0,0,0,0,0,0]], dtype=np.float)).reshape(2,6)
    P = np.diag(np.array([100, 100, 100, 1e-5, 1e-6, 1e-5], dtype=np.float)) # Initial Covariance Estimate of State
    
    traj, P, K = run(0, moonEph, sunEph, traj, P)
    print(traj, P, K)

if __name__ == "__main__":
    main()

