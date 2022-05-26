import pandas as pd
import numpy as np
import os.path
import json

from pandas.testing import assert_frame_equal
from const import TEST_C1_DISCRETIZED
from core.opnav import calculate_cam_measurements
from simulations.sim.src.opnav_sim import run_opnav_sim
from utils.constants import FLIGHT_SOFTWARE_PATH

SIM_DIR = os.path.join(FLIGHT_SOFTWARE_PATH, "OpticalNavigation/simulations/sim")


def main(traj_path: str, dt: int) -> None:
    """
    - Entry point function
    - Run the trajectory generation in order of:
        - prepare opnav trajectory for the sim
        - run opnav-sim
        - process opnav-sim outputs
    - End result is the UKF ready trajectory
    """

    print("Pre-processing trajectory for opnav-sim...")
    prepare_opnav_trajectory(traj_path, dt)

    print("Running opnav-sim on pre-processed trajectory...")
    run_opnav_sim(os.path.join(traj_path, 'pre_opnav'), False)

    # where observations.json is located inside sim directory
    path_to_opnav_obs = os.path.join(SIM_DIR, "data", "pre_opnav_sim")
    # where pre_opnav trajectory is located
    path_to_pre_opnav_csv = os.path.join(traj_path, 'pre_opnav')

    print("Processing opnav-sim results & creating final trajectory...")
    process_opnav_obs(path_to_opnav_obs, path_to_pre_opnav_csv)


def prepare_opnav_trajectory(traj_path: str, dt: int) -> None:
    """
    - The opnav-sim takes columns [t,x,y,z,vx,vy,vz,mx,my,mz,sx,sy,sz]
    - This will process partial trajectory files into one input for opnav-sim
    - x,y,z....vz from d_traj, mx...mz from mooneph, sx...sz from suneph
    """

    # traj files inside path
    path_dtraj = os.path.join(traj_path, 'trajectory', 'trajectory.csv')
    path_moonEph = os.path.join(traj_path, 'ephemeris', 'moon_eph.csv')
    path_sunEph = os.path.join(traj_path, 'ephemeris', 'sun_eph.csv')

    d_traj = pd.read_csv(path_dtraj)  # x,y,z, vx..vz
    d_moonEph = pd.read_csv(path_moonEph)
    d_sunEph = pd.read_csv(path_sunEph)

    # copy moonEph to final traj with opnav column names
    d_traj['mx'] = d_moonEph['x']
    d_traj['my'] = d_moonEph['y']
    d_traj['mz'] = d_moonEph['z']

    # same for sunEph, as sx..sz
    d_traj['sx'] = d_sunEph['x']
    d_traj['sy'] = d_sunEph['y']
    d_traj['sz'] = d_sunEph['z']

    # convert all positionals from km to m for opnav
    d_traj = d_traj.apply(lambda col: col*(10**3))

    # only add time column if user asks with a specific dt
    if dt != -1:
        # add length(traj) steps with dt
        num_rows = d_traj.shape[0]
        t_lst = [i*dt for i in range(num_rows)]
        d_traj.insert(0, column='t', value=t_lst)

    # new "pre_opnav" trajectory should be what opnav-sim expects
    d_traj.to_csv(os.path.join(traj_path, 'pre_opnav.csv'),
                  float_format="%e",
                  index=False)


def process_opnav_obs(path_to_opnav_result: str,
                      path_pre_opnav_csv: str) -> None:

    """
    - Parse through observations.json from the opnav-sim output
    - Insert angular measuremtns back into trajectory
    - Convert trajectory positionals back to km for UKF
    """

    opnav_result_file = os.path.join(path_to_opnav_result, "observations.json")
    with open(opnav_result_file, "r") as f:
        data = f.read()

    obj = json.loads(data)
    print("Total opnav observations: ", len(obj['observations']))

    ang_sep = [[], [], []]  # em, es, ms
    ang_size = [[], [], []]  # e, m, s

    # get angular data at time t, append it to ang_sep and ang_size sub-lists
    for i in range(len(obj['observations'])):
        # print("current obs: ", obj['observations'][i]['time'])
        e = (obj['observations'][i]['observed_bodies'][0])
        m = (obj['observations'][i]['observed_bodies'][1])
        s = (obj['observations'][i]['observed_bodies'][2])

        e_dir = e['direction_body']
        m_dir = m['direction_body']
        s_dir = s['direction_body']

        ang_em = calculate_cam_measurements(e_dir, m_dir)
        ang_es = calculate_cam_measurements(e_dir, s_dir)
        ang_ms = calculate_cam_measurements(m_dir, s_dir)

        ang_sep[0].append(ang_em)
        ang_sep[1].append(ang_es)
        ang_sep[2].append(ang_ms)

        ang_size[0].append(e['angular_size'])
        ang_size[1].append(m['angular_size'])
        ang_size[2].append(s['angular_size'])

    # write angular data from opnav-sim back into input trajectory
    d_traj = pd.read_csv(path_pre_opnav_csv + ".csv")

    # convert traj positionals from m to km for UKF, ignore time col
    d_traj = d_traj.apply(lambda col: col / (10**3) if col.name != 't' else col)

    # angular sep in radians
    d_traj['z1'] = ang_sep[0]  # em
    d_traj['z2'] = ang_sep[1]  # es
    d_traj['z3'] = ang_sep[2]  # ms

    # body sizes in radians
    d_traj['z4'] = ang_size[0]  # e (size)
    d_traj['z5'] = ang_size[1]  # m (size)
    d_traj['z6'] = ang_size[2]  # s (size)

    d_traj.to_csv(path_pre_opnav_csv + "_ukf_ready.csv",
                  float_format="%e",
                  index=False)


def test_traj_generation() -> None:
    """
    Test for trajectory generation:
    - Run through the full program using the c1_discretized dataset
    - Compare end results columns with the initial dataset columns
    - [z1...z6] are tested through sim validation and recreation of
        previous UKF experiments using the same dataset
    """

    traj_path = TEST_C1_DISCRETIZED

    # run generation on c1 file if it doesn't exist
    main(TEST_C1_DISCRETIZED, 60)

    # load initial trajectory data
    path_dtraj = os.path.join(traj_path, 'trajectory', 'trajectory.csv')
    path_moonEph = os.path.join(traj_path, 'ephemeris', 'moon_eph.csv')
    path_sunEph = os.path.join(traj_path, 'ephemeris', 'sun_eph.csv')

    d_traj = pd.read_csv(path_dtraj)  # x,y,z, vx..vz
    d_moonEph = pd.read_csv(path_moonEph)
    d_sunEph = pd.read_csv(path_sunEph)

    # load final trajectory data
    path_ukf_ready_traj = os.path.join(traj_path, 'pre_opnav_ukf_ready.csv')
    ukf_traj = pd.read_csv(path_ukf_ready_traj)

    # compare columns of the initial and final spacecraft trajectories
    assert_frame_equal(ukf_traj, ukf_traj)
    assert_frame_equal(ukf_traj.iloc[:, 1:7], d_traj)
    print("Spacecraft trajectory columns are equal!")

    # same as above, but filter vx..vz and ignore column names
    moonEphA = ukf_traj.iloc[:, 7:10]
    moonEphB = d_moonEph.iloc[:, 0:3]
    np.testing.assert_equal(moonEphA.values, moonEphB.values)
    print("Moon ephemeris columns are equal!")

    sunEphA = ukf_traj.iloc[:, 10:13]
    sunEphB = d_sunEph.iloc[:, 0:3]
    np.testing.assert_equal(sunEphA.values, sunEphB.values)
    print("Sun ephemeris columns are equal!")

    # lastly ensure time column hasn't changed
    time_list = np.arange(120)*60
    np.testing.assert_equal(ukf_traj.iloc[:, 0:1].values,
                            time_list.reshape(120, 1))
    print("Time columns are equal!")


if __name__ == "__main__":
    # main(TEST_C1_DISCRETIZED, 60)
    test_traj_generation()
