from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainSatelliteThread
    # for an explanation of the above 4 lines of code, see
    # https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
    # It lets your IDE know what type(self._parent) is, without causing any circular imports at runtime.

import gc
from time import sleep, time
from multiprocessing import Process
import subprocess
from queue import Empty
from traceback import format_tb

from utils.constants import *

import utils.parameters as params
from utils.log import get_log

from utils.exceptions import UnknownFlightModeException

no_transition_modes = [
    FMEnum.SensorMode.value,
    FMEnum.TestMode.value,
    FMEnum.Command.value
]
# this line of code brought to you by https://stackoverflow.com/questions/29503339/
all_modes = list(map(int, FMEnum))

logger = get_log()

NO_ARGS = ([], 0)


class FlightMode:
    # Override in Subclasses to tell CommandHandler the functions and arguments this flight mode takes
    command_codecs = {}
    sensordata_codecs = {}
    downlink_codecs = {}

    # Map argument names to (packer,unpacker) tuples
    # This tells CommandHandler how to serialize the arguments for commands to this flight mode
    command_arg_types = {}
    sensordata_arg_unpackers = {}
    downlink_arg_types = {}

    flight_mode_id = -1  # Value overridden in FM's implementation

    def __init__(self, parent: MainSatelliteThread):
        self._parent = parent
        self.task_completed = False

    def update_state(self) -> int:
        """update_state returns the id of the flight mode that we want to change to, which is then used in main.py's
        update_state to update our flight mode. All flight modes have their own implementation of update_state, but this
         serves as a basis for which most other flight modes can build off of."""

        # I am not sure this will properly work, but shuld have little impact for software demo
        if self._parent.opnav_process.is_alive():
            try:
                self._parent.opnav_process.join(timeout=1)
                result = self._parent.opnav_proc_queue.get(timeout=1)
                logger.info("[OPNAV]: ", result)
            except Empty:
                if not self._parent.opnav_process.is_alive():
                    self._parent.opnav_process.terminate()
                    logger.info("[OPNAV]: Process Terminated")

        flight_mode_id = self.flight_mode_id

        if flight_mode_id not in all_modes:
            raise UnknownFlightModeException(flight_mode_id)

        if flight_mode_id in no_transition_modes:
            return flight_mode_id

        # go to maneuver mode if there is something in the maneuver queue
        if not self._parent.maneuver_queue.empty() or params.SCHEDULED_BURN_TIME:
            if params.SCHEDULED_BURN_TIME > time():
                if params.SCHEDULED_BURN_TIME - time() < (60.0 * BURN_WAIT_TIME):
                    return FMEnum.Maneuver.value

        # go to reorientation mode if there is something in the reorientation queue
        if (not self._parent.reorientation_queue.empty()) or self._parent.reorientation_list:
            return FMEnum.AttitudeAdjustment.value

        # go to comms mode if there is something in the comms queue to downlink

        if not self._parent.downlink_queue.empty():
            return FMEnum.CommsMode.value

        # if battery is low, go to low battery mode
        batt_voltage = self._parent.telemetry.gom.hk.vbatt
        if (batt_voltage < params.ENTER_LOW_BATTERY_MODE_THRESHOLD) \
                and not params.IGNORE_LOW_BATTERY:
            return FMEnum.LowBatterySafety.value

        # if there is no current coming into the batteries, go to low battery mode
        if sum(self._parent.telemetry.gom.hk.curin) < params.ENTER_ECLIPSE_MODE_CURRENT \
                and batt_voltage < params.ENTER_ECLIPSE_MODE_THRESHOLD \
                and not params.IGNORE_LOW_BATTERY:
            return FMEnum.LowBatterySafety.value

        if self.task_completed:
            if self._parent.FMQueue.empty():
                return FMEnum.Normal.value
            else:
                return self._parent.FMQueue.get()

        return NO_FM_CHANGE  # returns -1 if the logic here does not make any FM changes

    def run_mode(self):
        raise NotImplementedError("Only implemented in specific flight mode subclasses")

    def execute_commands(self):
        bogus = bool()
        if len(self._parent.commands_to_execute) == 0:
            pass  # If I have no commands to execute do nothing
        else:
            # loop through commands in commands_to_execute list
            finished_commands = []

            for command in self._parent.commands_to_execute:

                bogus = False
                mac, counter, command_fm, command_id, command_kwargs = self._parent.command_handler.unpack_command(
                    command)
                logger.info(f"Received command {command_fm}:{command_id} with args {str(command_kwargs)}")

                try:
                    assert command_fm in self._parent.command_definitions.COMMAND_DICT
                    assert command_id in self._parent.command_definitions.COMMAND_DICT[command_fm]
                except AssertionError:
                    logger.warning(f"Rejecting bogus command {command_fm}:{command_id}:{command_kwargs}")
                    bogus = True

                if not bogus:
                    # changes the flight mode if command's FM is different.
                    if command_fm != self.flight_mode_id:
                        self._parent.replace_flight_mode_by_id(command_fm)

                    # locate which method to run:
                    method_to_run = self._parent.command_definitions.COMMAND_DICT[command_fm][command_id]
                    downlink_args = method_to_run(**command_kwargs)  # run that method, return downlink data

                    # Pack downlink given what the command returned
                    if downlink_args is not None:
                        downlink = self._parent.downlink_handler.pack_downlink(
                            self._parent.downlink_counter, command_fm, command_id,
                            **downlink_args)
                        self._parent.downlink_queue.put(downlink)

                finished_commands.append(command)

                # Prioritize downlinking: execute all necessary downlinks before
                # Starting next command
                # TODO: Execute downlinks before moving on to next command

            # TODO: Add try/except/finally statement above so that the for loop below always runs, even if an
            #  exception occurs in the above for loop
            for finished_command in finished_commands:
                self._parent.commands_to_execute.remove(finished_command)

    def poll_inputs(self):
        if self._parent.gom is not None:
            self._parent.gom.tick_wdt()  # FIXME; we don't want this for flight
            # The above line "pets" the dedicated watchdog timer on the GOMSpace P31u. This is an operational bug
            # An idea is to only pet the watchdog every time we recieve a command from the ground
        self._parent.telemetry.poll()

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
        logger.debug(f"Starting flight mode {self.flight_mode_id}")
        return self

    def __exit__(self, exc_type, exc_value, tb):
        logger.debug(f"Finishing flight mode {self.flight_mode_id}")
        if exc_type is not None:
            logger.error(f"Flight Mode failed with error type {exc_type} and value {exc_value}")
            logger.error(f"Failed with traceback:\n {format_tb(tb)}")


# Model for FlightModes that require precise timing
# Pause garbage collection and anything else that could
# interrupt critical thread
class PauseBackgroundMode(FlightMode):
    def run_mode(self):
        super().run_mode()

    def __init__(self, parent):
        super().__init__(parent)

    def __enter__(self):
        super().__enter__()
        self._parent.nemo_manager.pause()
        # TODO: Pause Opnav process if running
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()
        self._parent.nemo_manager.resume()
        # TODO: Resume Opnav process if previously running
        super().__exit__(exc_type, exc_val, exc_tb)


class TestMode(PauseBackgroundMode):
    """FMID 8: Testing Flight Mode
    Used to run tests while software is still in development. Could potentially be used for on-orbit testing."""

    flight_mode_id = FMEnum.TestMode.value

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self) -> int:
        return NO_FM_CHANGE  # this is intentional - we don't want the FM to update if we are testing something

    def run_mode(self):
        pass

    command_codecs = {TestCommandEnum.SeparationTest.value: ([], 0),
                      TestCommandEnum.ADCTest.value: ([], 0),
                      TestCommandEnum.CommsDriver.value: ([], 0),
                      TestCommandEnum.PiShutdown.value: ([], 0),
                      TestCommandEnum.RTCTest.value: ([], 0),
                      TestCommandEnum.LongString.value: (['some_number', 'long_string'], 180)
                      }

    command_arg_types = {
        'some_number': 'float',
        'long_string': 'string'
    }

    downlink_codecs = {TestCommandEnum.CommsDriver.value: (['gyro1', 'gyro2', 'gyro3'], 12)}

    downlink_arg_unpackers = {
        'gyro1': 'float',
        'gyro2': 'float',
        'gyro3': 'float',
    }


class CommsMode(FlightMode):
    """FMID 9: Downlinking Flight Mode
    This flight mode is dedicated to managing the radio and power systems to transmit data from the spacecraft
    to the ground station. This involves getting the correct RF switch direction, turning on the RF power amplifier,
    and transmitting a signal from the RF Board."""

    flight_mode_id = FMEnum.CommsMode.value
    command_codecs = {CommsCommandEnum.Switch.value: NO_ARGS}

    def __init__(self, parent):
        super().__init__(parent)
        self.electrolyzing = False

    def enter_transmit_safe_mode(self):

        # Stop electrolyzing
        if self._parent.gom.is_electrolyzing():
            self.electrolyzing = True
            self._parent.gom.set_electrolysis(False)

        # Set RF receiving side to low
        self._parent.gom.rf_receiving_switch(receive=False)

        # Turn off LNA
        self._parent.gom.lna(False)

        # Set RF transmitting side to high
        self._parent.gom.rf_transmitting_switch(receive=False)

        # Turn on power amplifier
        self._parent.gom.set_pa(on=True)

    def exit_transmit_safe_mode(self):

        # Turn off power amplifier
        self._parent.gom.set_pa(on=False)

        # Set RF transmitting side to low
        self._parent.gom.rf_transmitting_switch(receive=True)

        # Turn on LNA
        self._parent.gom.lna(True)

        # Set RF receiving side to high
        self._parent.gom.rf_receiving_switch(receive=True)

        # Resume electrolysis if we paused it to transmit
        if self.electrolyzing:
            self._parent.gom.set_electrolysis(True, delay=params.DEFAULT_ELECTROLYSIS_DELAY)

    def execute_downlinks(self):
        while not self._parent.downlink_queue.empty():
            self._parent.radio.transmit(self._parent.downlink_queue.get())
            self._parent.downlink_counter += 1
            sleep(params.DOWNLINK_BUFFER_TIME)  # TODO: revisit and see if we actually need this

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm
        return NO_FM_CHANGE

    def run_mode(self):
        if not self._parent.downlink_queue.empty():
            self.enter_transmit_safe_mode()
            self.execute_downlinks()
            self.exit_transmit_safe_mode()

        self.completed_task()


class OpNavMode(FlightMode):
    """FMID 5: Optical Navigation Flight Mode
    This flight mode is dedicated to starting the Opnav process"""
    # TODO: Flight Software/Opnav interface
    flight_mode_id = FMEnum.OpNav.value
    command_codecs = {OpNavCommandEnum.Switch.value: NO_ARGS}

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        # TODO: overhaul so opnav calculation only occur while in opnav mode
        if not self._parent.opnav_process.is_alive():
            logger.info("[OPNAV]: Able to run next opnav")
            self._parent.last_opnav_run = time()
            logger.info("[OPNAV]: Starting opnav subprocess")
            self._parent.opnav_process = Process(target=self.opnav_subprocess, args=(self._parent.opnav_proc_queue,))
            self._parent.opnav_process.start()
        self.completed_task()

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm

        # check if opnav db has been updated, then set self.task_completed true

        return NO_FM_CHANGE

    def opnav_subprocess(self, q):
        # TODO change from pytest to actual opnav
        # os.system("pytest OpticalNavigation/tests/test_pipeline.py::test_start")
        # subprocess.run('pytest OpticalNavigation/tests/test_pipeline.py::test_start', shell=True)
        subprocess.run('echo [OPNAV]: Subprocess Start; sleep 1m; echo [OPNAV]: Subprocess end', shell=True)
        q.put("Opnav Finished")


class SensorMode(FlightMode):
    """FMID 7: Sensor Mode
    This flight mode is not really well defined and has kinda been forgotten about. One potential idea for it is that we
    go into this mode whenever we want a high poll rate of all of our sensors to collect sensor info at as high of a
    rate as we can."""
    flight_mode_id = FMEnum.SensorMode.value
    command_codecs = {SensorsCommandEnum.Switch.value: NO_ARGS}

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError

    def update_state(self) -> int:
        return NO_FM_CHANGE  # intentional: we don't want to update FM when testing sensors


class ManeuverMode(PauseBackgroundMode):
    """FMID 6: Maneuver Mode
    This flight mode is dedicated to accurately firing our electrolysis thruster to make orbital changes"""
    flight_mode_id = FMEnum.Maneuver.value
    command_codecs = {ManeuverCommandEnum.Switch.value: NO_ARGS}
    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self) -> int:
        if self.task_completed is True:
            logger.info("Maneuver complete. Exiting maneuver mode...")
            return FMEnum.Normal.value
        return NO_FM_CHANGE

    def run_mode(self):
        # sleeping for 5 fewer seconds than the delay for safety
        # TODO: clear value of params.SCHEDULE_BURN_TIME after completion of burn
        sleep((params.SCHEDULED_BURN_TIME - time()) - 5)
        logger.info("Heating up glowplug to execute a maneuver...")
        # TODO: poll and check accelerometer values. If not acceleration seen, try other glowplug
        self._parent.gom.glowplug(GLOWPLUG_DURATION)
        self._parent.maneuver_queue.get()
        self.task_completed = True


# TODO
class SafeMode(FlightMode):
    """FMID 4: Safe Mode
    This flight mode is where we go to if there is any software fault during flight, or there is something awry."""
    flight_mode_id = FMEnum.Safety.value
    command_codecs = {SafetyCommandEnum.Switch.value: NO_ARGS,
                      SafetyCommandEnum.DetailedTelem.value: NO_ARGS,
                      SafetyCommandEnum.CritTelem.value: NO_ARGS,
                      SafetyCommandEnum.BasicTelem.value: NO_ARGS,
                      SafetyCommandEnum.ExitSafetyMode.value: NO_ARGS,
                      SafetyCommandEnum.SetParameter.value: ([NAME, VALUE, HARD_SET], 33), }

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        # TODO
        logger.info("Execute safe mode")


class NormalMode(FlightMode):
    flight_mode_id = FMEnum.Normal.value

    command_codecs = {
        NormalCommandEnum.Switch.value: NO_ARGS,
        NormalCommandEnum.RunOpNav.value: NO_ARGS,
        # NormalCommandEnum.SetDesiredAttitude.value: ([AZIMUTH, ELEVATION], 8),
        # NormalCommandEnum.SetAccelerate.value: ([ACCELERATE], 1),
        # NormalCommandEnum.SetBreakpoint.value: NO_ARGS,  # TODO define exact parameters
        NormalCommandEnum.SetParam.value: ([NAME, VALUE, HARD_SET], 33),
        NormalCommandEnum.CritTelem.value: NO_ARGS,
        NormalCommandEnum.BasicTelem.value: NO_ARGS,
        NormalCommandEnum.DetailedTelem.value: NO_ARGS,
        NormalCommandEnum.SetElectrolysis.value: ([STATE, DELAY], 5),
        NormalCommandEnum.SetOpnavInterval.value: ([INTERVAL], 4),
        NormalCommandEnum.Verification.value: ([NUM_BLOCKS], 2),
        NormalCommandEnum.GetParam.value: ([INDEX], 2),
        NormalCommandEnum.ScheduleManeuver.value: ([TIME], 4),
        NormalCommandEnum.ACSPulsing.value: ([START, PULSE_DURATION, PULSE_NUM, PULSE_DT], 14),
        NormalCommandEnum.NemoWriteRegister.value: ([REG_ADDRESS, REG_VALUE], 2),
        NormalCommandEnum.NemoReadRegister.value: ([REG_ADDRESS, REG_SIZE], 2),
        NormalCommandEnum.NemoSetConfig.value: ([
                                                    DET_ENABLE_UINT8,
                                                    DET0_BIAS_UINT8,
                                                    DET1_BIAS_UINT8,
                                                    DET0_THRESHOLD_UINT8,
                                                    DET1_THRESHOLD_UINT8,
                                                    RATE_WIDTH_MIN,
                                                    RATE_WIDTH_MAX,
                                                    BIN_WIDTH,
                                                    BIN_0_MIN_WIDTH,
                                                    RATE_INTERVAL,
                                                    VETO_THRESHOLD_MIN,
                                                    VETO_THRESHOLD_MAX,
                                                    CONFIG_WRITE_PERIOD,
                                                    CONFIG_ROTATE_PERIOD,
                                                    DATE_WRITE_PERIOD,
                                                    RATE_DATA_ROTATE_PERIOD,
                                                    HISTOGRAM_ROTATE_PERIOD,
                                                ], 32),
        NormalCommandEnum.NemoPowerOff.value: NO_ARGS,
        NormalCommandEnum.NemoPowerOn.value: NO_ARGS,
        NormalCommandEnum.NemoReboot.value: NO_ARGS,
        NormalCommandEnum.NemoProcessRateData.value: ([T_START, T_STOP, DECIMATION_FACTOR], 9),
        NormalCommandEnum.NemoProcessHistograms.value: ([T_START, T_STOP, DECIMATION_FACTOR], 9),
        NormalCommandEnum.GomConf1Set.value: ([PPT_MODE, BATTHEATERMODE, BATTHEATERLOW, BATTHEATERHIGH, OUTPUT_NORMAL1,
                                               OUTPUT_NORMAL2, OUTPUT_NORMAL3, OUTPUT_NORMAL4, OUTPUT_NORMAL5,
                                               OUTPUT_NORMAL6, OUTPUT_NORMAL7, OUTPUT_NORMAL8,
                                               OUTPUT_SAFE1,
                                               OUTPUT_SAFE2, OUTPUT_SAFE3, OUTPUT_SAFE4, OUTPUT_SAFE5, OUTPUT_SAFE6,
                                               OUTPUT_SAFE7, OUTPUT_SAFE8, OUTPUT_ON_DELAY, OUTPUT_OFF_DELAY, VBOOST1,
                                               VBOOST2, VBOOST3], 30),
        # TODO: clarify how many bytes go into string here
        NormalCommandEnum.ShellCommand.value: ([CMD], 24),
        NormalCommandEnum.SudoCommand.value: ([CMD], 24),
        NormalCommandEnum.Picberry.value: ([CMD], 24),
        NormalCommandEnum.GomConf1Get.value: NO_ARGS,
        NormalCommandEnum.GomConf2Set.value: ([MAX_VOLTAGE, NORM_VOLTAGE, SAFE_VOLTAGE, CRIT_VOLTAGE], 8),
        NormalCommandEnum.GomConf2Get.value: NO_ARGS,
        NormalCommandEnum.ExecPyFile.value: ([FNAME], 36),
        NormalCommandEnum.IgnoreLowBatt.value: ([IGNORE], 1)
    }

    command_arg_types = {
        AZIMUTH: 'float',
        ELEVATION: 'float',
        ACCELERATE: 'bool',
        NAME: 'string',
        VALUE: 'double',
        STATE: 'bool',
        INTERVAL: 'int',
        DELAY: 'short',
        NUM_BLOCKS: 'short',
        HARD_SET: 'bool',
        START: 'double',
        PULSE_DURATION: 'short',
        PULSE_NUM: 'short',
        PULSE_DT: 'short',
        TIME: 'float',
        REG_ADDRESS: 'uint8',
        REG_VALUE: 'uint8',
        REG_SIZE: 'uint8',
        DET_ENABLE_UINT8: 'uint8',
        DET0_BIAS_UINT8: 'uint8',
        DET1_BIAS_UINT8: 'uint8',
        DET0_THRESHOLD_UINT8: 'uint8',
        DET1_THRESHOLD_UINT8: 'uint8',
        RATE_WIDTH_MIN: 'uint8',
        RATE_WIDTH_MAX: 'uint8',
        BIN_WIDTH: 'uint8',
        BIN_0_MIN_WIDTH: 'uint8',
        RATE_INTERVAL: 'uint8',
        VETO_THRESHOLD_MIN: 'uint8',
        VETO_THRESHOLD_MAX: 'uint8',
        CONFIG_WRITE_PERIOD: 'int',
        CONFIG_ROTATE_PERIOD: 'int',
        DATE_WRITE_PERIOD: 'int',
        RATE_DATA_ROTATE_PERIOD: 'int',
        HISTOGRAM_ROTATE_PERIOD: 'int',
        T_START: 'int',
        T_STOP: 'int',
        DECIMATION_FACTOR: 'uint8',
        PPT_MODE: "uint8",
        BATTHEATERMODE: "bool",
        BATTHEATERLOW: "uint8",
        BATTHEATERHIGH: "uint8",
        OUTPUT_NORMAL1: "bool",
        OUTPUT_NORMAL2: "bool",
        OUTPUT_NORMAL3: "bool",
        OUTPUT_NORMAL4: "bool",
        OUTPUT_NORMAL5: "bool",
        OUTPUT_NORMAL6: "bool",
        OUTPUT_NORMAL7: "bool",
        OUTPUT_NORMAL8: "bool",
        OUTPUT_SAFE1: "bool",
        OUTPUT_SAFE2: "bool",
        OUTPUT_SAFE3: "bool",
        OUTPUT_SAFE4: "bool",
        OUTPUT_SAFE5: "bool",
        OUTPUT_SAFE6: "bool",
        OUTPUT_SAFE7: "bool",
        OUTPUT_SAFE8: "bool",
        OUTPUT_ON_DELAY: "short", OUTPUT_OFF_DELAY: "short",
        VBOOST1: "short", VBOOST2: "short", VBOOST3: "short",
        MAX_VOLTAGE: 'short',
        NORM_VOLTAGE: 'short',
        SAFE_VOLTAGE: 'short',
        CRIT_VOLTAGE: 'short',
        FNAME: 'string',
        CMD: 'string', IGNORE: 'bool',
    }

    downlink_codecs = {
        NormalCommandEnum.BasicTelem.value: ([RTC_TIME, ATT_1, ATT_2, ATT_3, ATT_4,
                                              HK_TEMP_1, HK_TEMP_2, HK_TEMP_3, HK_TEMP_4, GYRO_TEMP, THERMOCOUPLE_TEMP,
                                              CURRENT_IN_1, CURRENT_IN_2, CURRENT_IN_3,
                                              VBOOST_1, VBOOST_2, VBOOST_3, SYSTEM_CURRENT, BATTERY_VOLTAGE,
                                              PROP_TANK_PRESSURE], 84),

        NormalCommandEnum.SetParam.value: ([SUCCESSFUL], 1),
        NormalCommandEnum.GomConf1Set.value: command_codecs.get(NormalCommandEnum.GomConf1Set.value),
        NormalCommandEnum.ShellCommand.value: ([RETURN_CODE], 1)
    }

    downlink_arg_types = {
        RTC_TIME: 'double',
        POSITION_X: 'double',
        POSITION_Y: 'double',
        POSITION_Z: 'double',
        ATT_1: 'float',
        ATT_2: 'float',
        ATT_3: 'float',
        ATT_4: 'float',
        HK_TEMP_1: 'short',
        HK_TEMP_2: 'short',
        HK_TEMP_3: 'short',
        HK_TEMP_4: 'short',
        GYRO_TEMP: 'float',
        THERMOCOUPLE_TEMP: 'float',
        CURRENT_IN_1: 'short',
        CURRENT_IN_2: 'short',
        CURRENT_IN_3: 'short',
        VBOOST_1: 'short',
        VBOOST_2: 'short',
        VBOOST_3: 'short',
        SYSTEM_CURRENT: 'short',
        BATTERY_VOLTAGE: 'short',
        PROP_TANK_PRESSURE: 'float',
        SUCCESSFUL: 'bool',
        PPT_MODE: "uint8",
        BATTHEATERMODE: "bool",
        BATTHEATERLOW: "uint8",
        BATTHEATERHIGH: "uint8",
        OUTPUT_NORMAL1: "bool",
        OUTPUT_NORMAL2: "bool",
        OUTPUT_NORMAL3: "bool",
        OUTPUT_NORMAL4: "bool",
        OUTPUT_NORMAL5: "bool",
        OUTPUT_NORMAL6: "bool",
        OUTPUT_NORMAL7: "bool",
        OUTPUT_NORMAL8: "bool",
        OUTPUT_SAFE1: "bool",
        OUTPUT_SAFE2: "bool",
        OUTPUT_SAFE3: "bool",
        OUTPUT_SAFE4: "bool",
        OUTPUT_SAFE5: "bool",
        OUTPUT_SAFE6: "bool",
        OUTPUT_SAFE7: "bool",
        OUTPUT_SAFE8: "bool",
        OUTPUT_ON_DELAY: "short", OUTPUT_OFF_DELAY: "short",
        VBOOST1: "short", VBOOST2: "short", VBOOST3: "short",
        MAX_VOLTAGE: 'short',
        NORM_VOLTAGE: 'short',
        SAFE_VOLTAGE: 'short',
        CRIT_VOLTAGE: 'short',
        RETURN_CODE: "uint8"
    }

    def __init__(self, parent):
        super().__init__(parent)

    def poll_inputs(self):
        super().poll_inputs()

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm

        time_for_opnav: bool = (time() - self._parent.last_opnav_run) // 60 < params.OPNAV_INTERVAL
        time_for_telem: bool = (time() - self._parent.radio.last_transmit_time) // 60 < params.TELEM_INTERVAL
        need_to_electrolyze: bool = self._parent.telemetry.prs.pressure < params.IDEAL_CRACKING_PRESSURE
        currently_electrolyzing = self._parent.telemetry.gom.is_electrolyzing

        # if we don't want to electrolyze (per GS command), set need_to_electrolyze to false
        need_to_electrolyze = need_to_electrolyze and params.WANT_TO_ELECTROLYZE

        # if we don't want to run opnav (per GS command), set time_for_opnav to false
        time_for_opnav = time_for_opnav and params.WANT_TO_OPNAV

        # if currently electrolyzing and over pressure, stop electrolyzing
        if currently_electrolyzing and not need_to_electrolyze:
            self._parent.gom.set_electrolysis(False)

        if currently_electrolyzing and need_to_electrolyze:
            pass  # we are already in the state we want to be in

        # if below pressure and not electrolyzing, start electrolyzing
        if not currently_electrolyzing and need_to_electrolyze:
            self._parent.gom.set_electrolysis(True)

        if not currently_electrolyzing and not need_to_electrolyze:
            pass  # we are already in the state we want to be in

        # note: at this point, the variable "need_to_electrolyze" is equivalent to the new state of the electrolyzer

        if time_for_opnav:
            logger.info("Time to run Opnav")
            self._parent.FMQueue.put(FMEnum.OpNav.value)

        if time_for_telem:
            # Add a standard packet to the downlink queue for our period telemetry beacon
            telem = self._parent.telemetry.standard_packet_dict()
            downlink = self._parent.downlink_handler.pack_downlink(
                self._parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.BasicTelem.value,
                **telem)

            self._parent.downlink_queue.put(downlink)
            logger.info("Added a standard telemetry packet to the downlink queue")

        return NO_FM_CHANGE

    def run_mode(self):
        logger.info(f"In NORMAL flight mode")
        self.completed_task()


class CommandMode(PauseBackgroundMode):
    """FMID 10; Command Mode: a Flight Mode that listens for and runs commands and nothing else."""

    flight_mode_id = FMEnum.Command.value

    command_codecs = {
        CommandCommandEnum.Switch.value: NO_ARGS,
        CommandCommandEnum.SetParam.value: ([NAME, VALUE, HARD_SET], 33),
        CommandCommandEnum.SetSystemTime: ([TIME], 8),
        CommandCommandEnum.RebootPi: NO_ARGS,
        CommandCommandEnum.RebootGom: NO_ARGS,
        CommandCommandEnum.PowerCycle: NO_ARGS,
        CommandCommandEnum.GomPin: ([OUTPUT_CHANNEL, STATE, DELAY], 4),
        CommandCommandEnum.GeneralCmd.value: ([CMD], 24),  # TODO
        CommandCommandEnum.GomGeneralCmd.value: ([CMD], 24),  # TODO
        CommandCommandEnum.CeaseComms.value: ([PASSWORD], 8),
        CommandCommandEnum.SetUpdatePath.value: ([FILE_PATH], 195 - MIN_COMMAND_SIZE),
        CommandCommandEnum.AddFileBlock.value: ([BLOCK_NUMBER, BLOCK_TEXT], 195 - MIN_COMMAND_SIZE),
        CommandCommandEnum.GetFileBlocksInfo.value: ([TOTAL_BLOCKS], 2),
        CommandCommandEnum.ActivateFile.value: ([TOTAL_BLOCKS], 2),
        CommandCommandEnum.ShellCommand.value: ([CMD], 24)
    }

    command_arg_types = {
        FILE_PATH: 'string',
        BLOCK_NUMBER: 'short',
        BLOCK_TEXT: 'string',
        TOTAL_BLOCKS: 'short',
        SYS_TIME: 'double',
        GOM_PIN_STATE: 'bool',
        GOM_PIN_DELAY: 'short',
        OUTPUT_CHANNEL: 'uint8',
        PASSWORD: 'long'
    }

    downlink_codecs = {
        CommandCommandEnum.AddFileBlock.value: ([SUCCESSFUL, BLOCK_NUMBER], 3),
        CommandCommandEnum.GetFileBlocksInfo.value: ([CHECKSUM, MISSING_BLOCKS], 195 - MIN_COMMAND_SIZE),
        CommandCommandEnum.ShellCommand.value: ([RETURN_CODE], 1)
    }

    downlink_arg_types = {
        SUCCESSFUL: 'bool',
        BLOCK_NUMBER: 'short',
        CHECKSUM: 'string',
        MISSING_BLOCKS: 'string'
    }

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self) -> int:
        # DO NOT TICK THE WDT
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm  #
        return NO_FM_CHANGE

    def run_mode(self):
        pass  # intentional

    def poll_inputs(self):
        # TODO
        pass
        # raise NotImplementedError  # only check the comms queue
