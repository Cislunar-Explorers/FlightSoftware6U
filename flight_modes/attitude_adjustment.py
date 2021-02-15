from quaternion.quaternion_time_series import integrate_angular_velocity
from quaternion.numpy_quaternion import quaternion
import numpy as np
from astropy.coordinates import spherical_to_cartesian
from typing import Tuple
from time import sleep, time
from utils.constants import FMEnum, DEG2RAD, NO_FM_CHANGE
from flight_modes.flight_mode import FlightMode


def quat_to_rotvec(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """Converts a 4-tuple representation of a quaternion into a 3-tuple rotation vector."""
    rotvec = q[1:] / np.linalg.norm(q[1:])
    return rotvec


def theta(v, w): return np.arccos(np.dot(v, w) / (np.linalg.norm(v) * np.linalg.norm(w)))


def C3(theta):
    sin = np.sin(theta)
    cos = np.cos(theta)

    return np.array([[cos, sin, 0],
                     [-sin, cos, 0],
                     [0, 0, 1]])


class AAMode(FlightMode):
    """Attitude adjustment flight mode"""

    flight_mode_id = FMEnum.AttitudeAdjustment.value
    command_codecs = {}
    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)
        # Since we can't control the magnitude of our spin vector, we only car about the two angles (in spherical
        # coordinates) that define the direction of our spin vector, which saves us some up/downlink space.
        self.current_spin_vec = tuple() * 3
        self.target_spin_vec = tuple() * 3
        self.latest_opnav_quat = tuple() * 4
        self.latest_opnav_time = float()

    def get_data(self):
        # get latest opnav location and gyro data
        self.parent.tlm.opn.poll()
        self.latest_opnav_quat = self.parent.tlm.opn.quat
        self.latest_opnav_time = self.parent.tlm.opn.acq_time
        self.current_spin_vec = quat_to_rotvec(self.latest_opnav_quat)
        self.parent.tlm.gyr.poll()
        self.parent.tlm.gyr.write()

        while not self.parent.reorientation_queue.empty():
            self.parent.reorientation_list.append(self.parent.reorientation_queue.get())

        if self.parent.reorientation_list:
            self.target_spin_vec = spherical_to_cartesian(1, *self.parent.reorientation_list[0])

    def dead_reckoning(self) -> Tuple[Tuple[float, quaternion], Tuple[float, float, float]]:
        """Attempts to determine the current attitude of the spacecraft by integrating the gyros (prone to error) from
        the last opnav run. Returns a quaternion of the estimated attitude"""

        # query gyro data between self.latests_opnav_time and time.now(), needs to include
        gyro_data = np.array([tuple() * 4])
        gyro_times, gyroxs, gyroys, gyrozs = gyro_data

        # integration for dead reckoning to get estimate of current attitude
        # warning: will be inaccurate
        # TODO: look into efficacy of using an EKF to fuse accelerometer and gyro data into angular position estimate
        t, R = integrate_angular_velocity((gyro_times, (gyroxs, gyroys, gyrozs) * DEG2RAD),
                                          self.latest_opnav_time, time(), R0=self.latest_opnav_quat)

        latest_orientation_estimate = (t[-1], R[-1])
        latest_gyro_data = (gyroxs[-1], gyroys[-1], gyrozs[-1])

        return latest_orientation_estimate, latest_gyro_data

    def calculate_firing_orientation(self, cur_quat: quaternion, desired_spin_axis: Tuple[float, float]):
        """Calculates the optimal vector along which to fire the ACS thruster, cur_spin_axis is a numpy quaternion of
        the current orientation and desired_spin_axis is a  2-tuple of floats of the angles of elevation (from the
        equator) and azimuth of the spin vectors in a spherical ECI frame. This function returns the optimal pointing
        location of the cislunar explorer while firing """

        cur_spin_vector = cur_quat.vec
        desired_spin_vector = spherical_to_cartesian(1, *desired_spin_axis)

        firing_vector_ECI = np.cross(cur_spin_vector, desired_spin_vector)

        gyroz = self.parent.tlm.gyr.rot(2)

        firing_vector_ECI *= gyroz / abs(gyroz)  # multiply by sign of z rotation rate to avoid sign error
        # firing_vector_ECI is the vector along which the vector from the CoM of the spacecraft to the ACS nozzle
        # shall fire around

        # everything below related to DCM should be precomputed and stored as a constant somewhere
        r_ACS_B = (0.08, -0.19, 0)  # Estimate for vector between CoM and ACS nozzle in spacecraft body-frame (in m)

        theta_ACS = theta(r_ACS_B, (1, 0, 0))
        DCM_ACS = C3(theta_ACS)

        return np.matmul(DCM_ACS, firing_vector_ECI)

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm

    # TODO implement actual maneuver execution
    # check if exit condition has completed
    def run_mode(self):

        outdated_opnav = (time() - self.parent.tlm.opn.acq_time) // 60 > 15  # if opnav was run >15 minutes ago, rerun
        unfinished_opnav = not self.parent.tlm.opn.currently_running()

        if outdated_opnav or unfinished_opnav:
            self.parent.FMQueue.put(FMEnum.OpNav.value)
            self.parent.FMQueue.put(FMEnum.AttitudeAdjustment.value)
        else:
            # check if current orientation is within 10 degrees of desired orientation
            orientation_estimate, gyro_data = self.dead_reckoning()

            self.calculate_firing_orientation(orientation_estimate[1], self.parent.reorientation_queue.get())
            # calculate firing times based off firing orientation and "current" orientation
            # stop garbage collection and other threads
            # send pulse commands
            # resume garbage collection and other threads
            # run opnav again

        self.moves_towards_goal()
        if self.moves_towards_goal >= self.goal:
            self.task_completed()
