from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainSatelliteThread

    # for an explanation of the above 4 lines of code, see
    # https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
    # It lets your IDE know what type(self._main) is, without causing any circular imports at runtime.

import gc
from time import sleep, time
from multiprocessing import Process
import subprocess
from queue import Empty
from traceback import format_tb
import logging

from fsw.utils import constants as consts
from fsw.utils import parameters as params

from fsw.utils.exceptions import UnknownFlightModeException

no_transition_modes = [
    consts.FMEnum.SensorMode.value,
    consts.FMEnum.TestMode.value,
    consts.FMEnum.Command.value,
]
# this line of code brought to you by https://stackoverflow.com/questions/29503339/
all_modes = list(map(int, consts.FMEnum))

NO_ARGS = ([], 0)


class FlightMode:
    flight_mode_id = -1  # Value overridden in FM's implementation

    def __init__(self, main: MainSatelliteThread):
        self._main = main
        self.task_completed = False

    def update_state(self, sim_input=None) -> int:
        """update_state returns the id of the flight mode that we want to change to, which is then used in main.py's
        update_state to update our flight mode. All flight modes have their own implementation of update_state, but
        this serves as a basis for which most other flight modes can build off of."""

        # If this is a sim run, there's currently no support for simulating Opnav.
        if sim_input is None:
            # I am not sure this will properly work, but shuld have little impact for software demo
            if self._main.opnav_process.is_alive():
                try:
                    self._main.opnav_process.join(timeout=1)
                    result = self._main.opnav_proc_queue.get(timeout=1)
                    logging.info("[OPNAV]: ", result)
                except Empty:
                    if not self._main.opnav_process.is_alive():
                        self._main.opnav_process.terminate()
                        logging.info("[OPNAV]: Process Terminated")

        flight_mode_id = self.flight_mode_id

        if flight_mode_id not in all_modes:
            raise UnknownFlightModeException(flight_mode_id)

        if flight_mode_id in no_transition_modes:
            return flight_mode_id

        # go to maneuver mode if there is something in the maneuver queue
        if not self._main.maneuver_queue.empty() or params.SCHEDULED_BURN_TIME:
            if params.SCHEDULED_BURN_TIME > time():
                if params.SCHEDULED_BURN_TIME - time() < (60.0 * consts.BURN_WAIT_TIME):
                    return consts.FMEnum.Maneuver.value
            else:
                logging.info(
                    f"Scheduled burn time at {params.SCHEDULED_BURN_TIME} has passed and will be skipped"
                )

        # go to reorientation mode if there is something in the reorientation queue
        if (
            not self._main.reorientation_queue.empty()
        ) or self._main.reorientation_list:
            return consts.FMEnum.AttitudeAdjustment.value

        # go to comms mode if there is something in the comms queue to downlink

        if not self._main.downlink_queue.empty():
            return consts.FMEnum.CommsMode.value

        if sim_input is None:
            # if battery is low, go to low battery mode
            batt_voltage = self._main.telemetry.gom.hk.vbatt
            if (
                batt_voltage < params.ENTER_LOW_BATTERY_MODE_THRESHOLD
            ) and not params.IGNORE_LOW_BATTERY:
                return consts.FMEnum.LowBatterySafety.value
            # if there is no current coming into the batteries, go to low battery mode
            if (
                sum(self._main.telemetry.gom.hk.curin)
                < params.ENTER_ECLIPSE_MODE_CURRENT
                and batt_voltage < params.ENTER_ECLIPSE_MODE_THRESHOLD
                and not params.IGNORE_LOW_BATTERY
            ):
                return consts.FMEnum.LowBatterySafety.value
        else:
            # TODO: Implement sim mocking of battery level
            batt_voltage = 0
        if self.task_completed:
            if self._main.FMQueue.empty():
                return consts.FMEnum.Normal.value
            else:
                return self._main.FMQueue.get()

        # returns -1 if the logic here does not make any FM changes
        return consts.NO_FM_CHANGE

    def run_mode(self):
        raise NotImplementedError("Only implemented in specific flight mode subclasses")

    def execute_commands(self):
        if len(self._main.commands_to_execute) == 0:
            pass  # If I have no commands to execute do nothing
        else:
            # loop through commands in commands_to_execute list
            finished_commands = []

            for command in self._main.commands_to_execute:
                downlink = self._main.command_handler.execute_command(command)
                if downlink is not None:
                    self._main.downlink_queue.put(downlink)

                finished_commands.append(command)

                # Prioritize downlinking: execute all necessary downlinks before
                # Starting next command
                # TODO: Execute downlinks before moving on to next command

            # TODO: Add try/except/finally statement above so that the for loop below always runs, even if an
            #  exception occurs in the above for loop
            for finished_command in finished_commands:
                self._main.commands_to_execute.remove(finished_command)

    def poll_inputs(self, sim_input=None):
        if sim_input is None:
            # TODO: Comment on what polling inputs means and how they differ across flight modes
            self._main.devices.gom.tick_wdt()  # FIXME; we don't want this for flight
            # The above line "pets" the dedicated watchdog timer on the GOMSpace P31u. This is an operational bug
            # An idea is to only pet the watchdog every time we recieve a command from the ground
            self._main.telemetry.poll()
        else:
            # TODO: process sim_sensory_data
            pass

    def completed_task(self):
        self.task_completed = True

    # Autonomous actions to maintain safety
    def automatic_actions(self):
        pass

    def write_telemetry(self):
        pass

    #    @staticmethod
    #    def update_mission_mode(cls, mission_mode_id):
    #        """Sets the behavior of flight modes. Acts like a meta-flight mode."""
    #        if mission_mode_id == MissionModeEnum.Boot.value:
    #            # set parameters for boot
    #            params.CURRENT_MISSION_MODE = 0
    #            params.WANT_TO_ELECTROLYZE = False
    #            params.WANT_TO_OPNAV = False
    #            params.TELEM_DOWNLINK_TIME = 10
    #
    #        if mission_mode_id == MissionModeEnum.Normal.value:
    #            # set parameters for normal mission mode
    #            params.CURRENT_MISSION_MODE = 1
    #            params.WANT_TO_ELECTROLYZE = True
    #            params.WANT_TO_OPNAV = True
    #            params.TELEM_DOWNLINK_TIME = 60

    def __enter__(self):
        logging.debug(f"Starting flight mode {self.flight_mode_id}")
        return self

    def __exit__(self, exc_type, exc_value, tb):
        logging.debug(f"Finishing flight mode {self.flight_mode_id}")
        if exc_type is not None:
            logging.error(
                f"Flight Mode failed with error type {exc_type} and value {exc_value}"
            )
            logging.error(f"Failed with traceback:\n {''.join(format_tb(tb))}")


class PauseBackgroundMode(FlightMode):
    """Model for FlightModes that require precise timing
    Pause garbage collection and anything else that could
    interrupt critical thread"""

    def run_mode(self):
        super().run_mode()

    def __init__(self, main):
        super().__init__(main)

    def __enter__(self):
        super().__enter__()
        if self._main.nemo_manager is not None:
            self._main.nemo_manager.pause()
        # TODO: Pause Opnav process if running
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()
        if self._main.nemo_manager is not None:
            self._main.nemo_manager.resume()
        # TODO: Resume Opnav process if previously running
        super().__exit__(exc_type, exc_val, exc_tb)


class TestMode(PauseBackgroundMode):
    """FMID 8: Testing Flight Mode
    Used to run tests while software is still in development. Could potentially be used for on-orbit testing."""

    flight_mode_id = consts.FMEnum.TestMode.value

    def __init__(self, main):
        super().__init__(main)

    def update_state(self) -> int:
        # this is intentional - we don't want the FM to update if we are testing something
        return consts.NO_FM_CHANGE

    def run_mode(self):
        pass


class CommsMode(FlightMode):
    """FMID 9: Downlinking Flight Mode
    This flight mode is dedicated to managing the radio and power systems to transmit data from the spacecraft
    to the ground station. This involves getting the correct RF switch direction, turning on the RF power amplifier,
    and transmitting a signal from the RF Board."""

    flight_mode_id = consts.FMEnum.CommsMode.value

    def __init__(self, main):
        super().__init__(main)
        self.electrolyzing = False

    def enter_transmit_safe_mode(self):
        # Check if hardware is available or if we should write to sim output instead
        if self._main.sim_output is None:
            # Stop electrolyzing
            if self._main.devices.gom.is_electrolyzing():
                self.electrolyzing = True
                self._main.devices.gom.electrolyzers.set(False)

            # Set RF receiving side to low
            self._main.devices.gom.rf_rx.set(False)

            # Turn off LNA
            self._main.devices.gom.lna.set(False)

            # Set RF transmitting side to high
            self._main.devices.gom.rf_tx.set(True)

            # Turn on power amplifier
            self._main.devices.gom.pa.set(True)
        else:
            # Writing hardware calls to sim instead

            # TODO: Sim out the check to devices.gom.is_electrolyzing()

            # Set RF receiving side to low
            self._main.sim_output.write_single_entry("gom.rf_rx", False)

            # Turn off LNA
            self._main.sim_output.write_single_entry("gom.lna", False)

            # Set RF transmitting side to high
            self._main.sim_output.write_single_entry("gom.rf_tx", True)

            # Turn on power amplifier
            self._main.sim_output.write_single_entry("gom.pa", True)

    def exit_transmit_safe_mode(self):
        # Check if hardware is available or if we should write to sim output instead
        if self._main.sim_output is None:
            # Turn off power amplifier
            self._main.devices.gom.pa.set(False)

            # Set RF transmitting side to low
            self._main.devices.gom.rf_tx.set(False)

            # Turn on LNA
            self._main.devices.gom.lna.set(True)

            # Set RF receiving side to high
            self._main.devices.gom.rf_rx.set(True)

            # Resume electrolysis if we paused it to transmit
            if self.electrolyzing:
                self._main.devices.gom.electrolyzers.set(
                    True, delay=params.DEFAULT_ELECTROLYSIS_DELAY
                )
        else:
            # Turn off power amplifier
            self._main.sim_output.write_single_entry("pa", False)

            # Set RF transmitting side to low
            self._main.sim_output.write_single_entry("rf_tx", False)

            # Turn on LNA
            self._main.sim_output.write_single_entry("lna", True)

            # Set RF receiving side to high
            self._main.sim_output.write_single_entry("rf_rx", True)

            # Resume electrolysis if we paused it to transmit
            # TODO: Mock check to self.electrolyzing

    def execute_downlinks(self):
        while not self._main.downlink_queue.empty():
            if self._main.sim_output is None:
                self._main.devices.radio.transmit(self._main.downlink_queue.get())
            else:
                self._main.sim_output.write_single_entry(
                    "radio.transmit", self._main.downlink_queue.get()
                )
            self._main.downlink_counter += 1
            # TODO: revisit and see if we actually need this
            sleep(params.DOWNLINK_BUFFER_TIME)

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != consts.NO_FM_CHANGE:
            return super_fm
        return consts.NO_FM_CHANGE

    def run_mode(self):
        if not self._main.downlink_queue.empty():
            self.enter_transmit_safe_mode()
            self.execute_downlinks()
            self.exit_transmit_safe_mode()

        self.completed_task()


class OpNavMode(FlightMode):
    """FMID 5: Optical Navigation Flight Mode
    This flight mode is dedicated to starting the Opnav process"""

    # TODO: Flight Software/Opnav interface
    flight_mode_id = consts.FMEnum.OpNav.value

    def __init__(self, main):
        super().__init__(main)

    def run_mode(self):
        # TODO: overhaul so opnav calculation only occur while in opnav mode
        if not self._main.opnav_process.is_alive():
            logging.info("[OPNAV]: Able to run next opnav")
            self._main.last_opnav_run = time()
            logging.info("[OPNAV]: Starting opnav subprocess")
            self._main.opnav_process = Process(
                target=self.opnav_subprocess, args=(self._main.opnav_proc_queue,)
            )
            self._main.opnav_process.start()
        self.completed_task()

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != consts.NO_FM_CHANGE:
            return super_fm

        # check if opnav db has been updated, then set self.task_completed true

        return consts.NO_FM_CHANGE

    def opnav_subprocess(self, q):
        # TODO put in try...except
        # TODO change from pytest to actual opnav
        # os.system("pytest opnav/tests/test_pipeline.py::test_start")
        # subprocess.run('pytest opnav/tests/test_pipeline.py::test_start', shell=True)
        subprocess.run(
            "echo [OPNAV]: Subprocess Start; sleep 1m; echo [OPNAV]: Subprocess end",
            shell=True,
        )
        q.put("Opnav Finished")


class SensorMode(FlightMode):
    """FMID 7: Sensor Mode
    This flight mode is not really well defined and has kinda been forgotten about. One potential idea for it is that
    we go into this mode whenever we want a high poll rate of all of our sensors to collect sensor info at as high of
    a rate as we can."""

    flight_mode_id = consts.FMEnum.SensorMode.value

    def __init__(self, main):
        super().__init__(main)

    def update_state(self) -> int:
        return (
            consts.NO_FM_CHANGE
        )  # intentional: we don't want to update FM when testing sensors

    def run_mode(self):
        raise NotImplementedError


# TODO
class SafeMode(FlightMode):
    """FMID 4: Safe Mode
    This flight mode is where we go to if there is any software fault during flight, or there is something awry."""

    flight_mode_id = consts.FMEnum.Safety.value

    def __init__(self, main):
        super().__init__(main)

    def run_mode(self):
        # TODO
        logging.info("Execute safe mode")


class NormalMode(FlightMode):
    flight_mode_id = consts.FMEnum.Normal.value

    def __init__(self, main):
        super().__init__(main)

    def poll_inputs(self):
        super().poll_inputs()

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != consts.NO_FM_CHANGE:
            return super_fm

        # since Normal mode never completes its task, we need to check the queue here instead of in super.update_state
        if not self._main.FMQueue.empty():
            return self._main.FMQueue.get()

        if self._main.sim_input is None:
            time_for_opnav: bool = (
                time() - self._main.last_opnav_run
            ) // 60 > params.OPNAV_INTERVAL
            time_for_telem: bool = (
                time() - self._main.devices.radio.last_transmit_time
            ) // 60 > params.TELEM_INTERVAL
        else:
            # TODO: Sim out opnav
            time_for_opnav: bool = False

            # TODO: Read from sim input/output radio.last_transmit_time
            # TODO: Sim out radio
            time_for_telem: bool = False

        need_to_electrolyze: bool = self._main.telemetry.prs.pressure < params.IDEAL_CRACKING_PRESSURE

        if self._main.sim_input is None:
            currently_electrolyzing = self._main.devices.gom.is_electrolyzing()
        else:
            # TODO: Sim out gom.is_electrolyzing
            currently_electrolyzing = False

        # if we don't want to electrolyze (per GS command), set need_to_electrolyze to false
        need_to_electrolyze = need_to_electrolyze and params.WANT_TO_ELECTROLYZE

        # if we don't want to run opnav (per GS command), set time_for_opnav to false
        time_for_opnav = time_for_opnav and params.WANT_TO_OPNAV

        # if currently electrolyzing and over pressure, stop electrolyzing
        if currently_electrolyzing and not need_to_electrolyze:
            logging.info("No need to electrolyze, turning OFF electrolyzers")
            if self._main.sim_output is None:
                self._main.devices.gom.electrolyzers.set(False)
            else:
                self._main.sim_output.write_single_entry("gom.electrolyzers", False)

        if currently_electrolyzing and need_to_electrolyze:
            logging.debug("Already electrolyzing")
            pass  # we are already in the state we want to be in

        # if below pressure and not electrolyzing, start electrolyzing
        if not currently_electrolyzing and need_to_electrolyze:
            logging.info("Not electrolyzing, turning ON electrolyzers")
            if self._main.sim_output is None:
                self._main.devices.gom.electrolyzers.set(True)
            else:
                self._main.sim_output.write_single_entry("gom.electrolyzers", True)

        if not currently_electrolyzing and not need_to_electrolyze:
            logging.debug("Electrolyzers already OFF")
            pass  # we are already in the state we want to be in

        # note: at this point, the variable "need_to_electrolyze" is equivalent to the new state of the electrolyzer

        if time_for_opnav:
            logging.info("Time to run opnav")
            self._main.FMQueue.put(consts.FMEnum.OpNav.value)

        if time_for_telem:
            # Add a standard packet to the downlink queue for our period telemetry beacon
            telem = self._main.telemetry.standard_packet_dict()
            downlink = self._main.command_handler.pack_telemetry(
                consts.CommandEnum.BasicTelem, telem
            )
            self._main.downlink_queue.put(downlink)
            logging.info("Added a standard telemetry packet to the downlink queue")

        return consts.NO_FM_CHANGE

    def run_mode(self):
        logging.info("In NORMAL flight mode")


class CommandMode(PauseBackgroundMode):
    """FMID 10; Command Mode: a Flight Mode that listens for and runs commands and nothing else."""

    flight_mode_id = consts.FMEnum.Command.value

    def __init__(self, main):
        super().__init__(main)

    def update_state(self) -> int:
        # DO NOT TICK THE WDT
        super_fm = super().update_state()
        if super_fm != consts.NO_FM_CHANGE:
            return super_fm  #
        return consts.NO_FM_CHANGE

    def run_mode(self):
        pass  # intentional

    def poll_inputs(self):
        # TODO
        pass
        # raise NotImplementedError  # only check the comms queue
