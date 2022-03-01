# from quaternion.numpy_quaternion import quaternion
# import numpy as np
# from typing import Tuple
from time import sleep, time
from utils.constants import FMEnum, GOM_TIMING_FUDGE_FACTOR
import utils.parameters as params
from flight_modes.flight_mode import PauseBackgroundMode
import logging

# from math import sin, cos
import gc

"""
def quat_to_rotvec(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    #Converts a 4-tuple representation of a quaternion into a 3-tuple rotation vector.
    rotvec = q[1:] / np.linalg.norm(q[1:])
    return rotvec


def spherical_to_cartesian(v: Tuple[float, float]) -> Tuple[float, float, float]:
    # Converts two angles (azimuth and elevation) into a normalized vector (length = 1).
    #   Angle of elevation is measured from the north pole down

    _theta = v[0]
    _phi = v[1]

    return sin(_phi) * cos(_theta), sin(_phi) * sin(_theta), cos(_phi)


def theta(v, w): return np.arccos(np.dot(v, w) / (np.linalg.norm(v) * np.linalg.norm(w)))


def C3(theta):
    sint = sin(theta)
    cost = cos(theta)

    return np.array([[cost, sint, 0],
                     [-sint, cost, 0],
                     [0, 0, 1]])
"""


class AAMode(PauseBackgroundMode):
    """FMID 11: Attitude adjustment flight mode"""

    flight_mode_id = FMEnum.AttitudeAdjustment.value
    command_codecs = {}
    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)
        # Since we can't control the magnitude of our spin vector, we only car about the two angles (in spherical
        # coordinates) that define the direction of our spin vector, which saves us some up/downlink space.
        # data for if we want to do autonomous attitude adjustment
        # self.current_spin_vec: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        # self.target_spin_vec: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        # self.latest_opnav_quat: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
        # self.latest_opnav_time = float()

    def update_state(self) -> int:
        return super().update_state()

    # check if exit condition has completed
    def run_mode(self):
        # double check to make sure that we actually have the info we need to reorient:
        if (
            not self._parent.reorientation_queue.empty()
            or self._parent.reorientation_list
        ):
            # get relevant info from attitude adjustment queue
            while not self._parent.reorientation_queue.empty():
                self._parent.reorientation_list.append(
                    self._parent.reorientation_queue.get()
                )

            (
                pulse_start,
                pulse_duration,
                pulse_num,
                pulse_dt,
            ) = self._parent.reorientation_list[0]
            logging.debug(
                f"ACS start:{pulse_start}, duration:{pulse_duration}, num:{pulse_num}, dt:{pulse_dt}"
            )
            pulse_dt *= 1e-3  # convert from ms to seconds
            pulse_duration *= 1e-3

            relative_pulse_times = [x * pulse_dt for x in range(pulse_num)]
            # https://www.geeksforgeeks.org/python-adding-k-to-each-element-in-a-list-of-integers/
            absolute_pulse_times = [pulse_start + x for x in relative_pulse_times]
            for i in absolute_pulse_times:
                logging.debug(f"Pulsing ACS at {i}")
            # garbage collect one last time before timing critical applications start
            gc.collect()
            logging.debug("Done garbage collecting")

            if pulse_start - time() < 0.125:
                # we missed the timing of the maneuver. Make a note and add relevant stuff to comms queue
                self.missed_timing(pulse_start)
            else:
                logging.debug(f"Sleeping {pulse_start - time()}s")
                self._parent.gom.driver.calculate_solenoid_wave()
                sleep(max([(pulse_start - time()) - 2, 0]))
                logging.info(
                    f"Experimental solenoid function. spike={params.ACS_SPIKE_DURATION}, hold={pulse_duration}"
                )
                # pulse ACS according to timings
                for pulse_time in absolute_pulse_times:
                    sleep((pulse_time - time()) - (GOM_TIMING_FUDGE_FACTOR * 1e-3))
                    self._parent.gom.driver.solenoid_single_wave(pulse_duration)
            self._parent.reorientation_list.pop(0)
        else:
            logging.warning("No data for reorientation pulses found")

        self.completed_task()

    def missed_timing(self, missed_pulse_timing):
        logging.error(f"Missed attitude adjustment maneuver at {missed_pulse_timing}")
        # self._parent.communications_queue.put((ErrorCodeEnum.MissedPulse.value, missed_pulse_timing))

    # the methods defined below are for autonomous reorientation (i.e. we send the spacecraft a new spin vector and
    # it does all the math. However, due to our development timeline, we will have to fall back on calculating exact
    # timings on the ground, and then the satellite blindly follows these timings
    """
    def get_data(self):
        # get latest opnav location and gyro data
        self._parent.tlm.opn.poll()
        self.latest_opnav_quat = self._parent.tlm.opn.quat
        self.latest_opnav_time = self._parent.tlm.opn.acq_time
        self.current_spin_vec = quat_to_rotvec(self.latest_opnav_quat)
        self._parent.tlm.gyr.poll()
        self._parent.tlm.gyr.write()

        while not self._parent.reorientation_queue.empty():
            self._parent.reorientation_list.append(self._parent.reorientation_queue.get())

        if self._parent.reorientation_list:
            self.target_spin_vec = spherical_to_cartesian(*self._parent.reorientation_list[0])

    def dead_reckoning(self) -> Tuple[Tuple[float, quaternion], Tuple[float, float, float]]:
        # Attempts to determine the current attitude of the spacecraft by integrating the gyros (prone to error) from
        # the last opnav run. Returns a quaternion of the estimated attitude"
        from quaternion.quaternion_time_series import integrate_angular_velocity

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

    def calculate_firing_orientation(self, cur_quat: quaternion, desired_spin_vector: Tuple[float, float, float]):
        # Calculates the optimal vector along which to fire the ACS thruster, cur_spin_axis is a numpy quaternion of
        # the current orientation and desired_spin_axis is a  2-tuple of floats of the angles of elevation (from the
        # equator) and azimuth of the spin vectors in a spherical ECI frame. This function returns the optimal pointing
        # location of the cislunar explorer while firing

        cur_spin_vector = cur_quat.vec
        firing_vector_ECI = np.cross(cur_spin_vector, desired_spin_vector)

        gyroz = self._parent.tlm.gyr.rot(2)

        firing_vector_ECI *= gyroz / abs(gyroz)  # multiply by sign of z rotation rate to avoid sign error
        # firing_vector_ECI is the vector along which the vector from the CoM of the spacecraft to the ACS nozzle
        # shall fire around

        # everything below related to DCM should be precomputed and stored as a constant somewhere
        r_ACS_B = (0.08, -0.19, 0)  # Estimate for vector between CoM and ACS nozzle in spacecraft body-frame (in m)

        theta_ACS = theta(r_ACS_B, (1, 0, 0))
        DCM_ACS = C3(theta_ACS)

        return np.matmul(DCM_ACS, firing_vector_ECI)

# If we want to do reorientations autonomously, the following would be an approach.
# However, we are probably not doing this
#
outdated_opnav = (time() - self._parent.tlm.opn.acq_time) // 60 >= 15  # if opnav was run >15 minutes ago, rerun
unfinished_opnav = not self._parent.tlm.opn.currently_running()

if outdated_opnav or unfinished_opnav:
    self._parent.FMQueue.put(FMEnum.OpNav.value)
    self._parent.FMQueue.put(FMEnum.AttitudeAdjustment.value)
else:
    # check if current orientation is within 10 degrees of desired orientation
    self.get_data()
    current = np.array(self.current_spin_vec)
    target = np.array(self.target_spin_vec)

    angle = np.arccos(np.dot(current, target))

    if angle < 10 * DEG2RAD:
        self.task_completed = True
    else:
        orientation_estimate, gyro_data = self.dead_reckoning()

        self.calculate_firing_orientation(orientation_estimate[1], self.target_spin_vec)
        # calculate firing times based off firing orientation and "current" orientation
        # stop garbage collection and other threads
        # send pulse commands
        # resume garbage collection and other threads
        # run opnav again
"""
