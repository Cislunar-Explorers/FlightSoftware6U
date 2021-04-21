from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainSatelliteThread
    # for an explanation of the above 4 lines of code, see
    # https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
    # It lets your IDE know what type(self.parent) is, without causing any circular imports at runtime.

import gc
from time import sleep, time
from datetime import datetime
from multiprocessing import Process
import subprocess
from queue import Empty

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
        self.parent = parent
        self.task_completed = False

    def update_state(self) -> int:
        """update_state returns the id of the flight mode that we want to change to, which is then used in main.py's
        update_state to update our flight mode. All flight modes have their own implementation of update_state, but this
         serves as a basis for which most other flight modes can build off of."""

        # I am not sure this will properly work, but shuld have little impact for software demo
        if self.parent.opnav_process.is_alive():
            try:
                self.parent.opnav_process.join(timeout=1)
                result = self.parent.opnav_proc_queue.get(timeout=1)
                logger.info("[OPNAV]: ", result)
            except Empty:
                if not self.parent.opnav_process.is_alive():
                    self.parent.opnav_process.terminate()
                    logger.info("[OPNAV]: Process Terminated")

        flight_mode_id = self.flight_mode_id

        if flight_mode_id not in all_modes:
            raise UnknownFlightModeException(flight_mode_id)

        if self.task_completed:
            if self.parent.FMQueue.empty():
                return FMEnum.Normal.value
            else:
                return self.parent.FMQueue.get()

        if flight_mode_id in no_transition_modes:
            return flight_mode_id

        # go to maneuver mode if there is something in the maneuver queue
        if not self.parent.maneuver_queue.empty() or params.SCHEDULED_BURN_TIME:
            if params.SCHEDULED_BURN_TIME > time():
                if params.SCHEDULED_BURN_TIME - time() < (60.0 * BURN_WAIT_TIME):
                    return FMEnum.Maneuver.value

        # go to reorientation mode if there is something in the reorientation queue
        if (not self.parent.reorientation_queue.empty()) or self.parent.reorientation_list:
            return FMEnum.AttitudeAdjustment.value

        # if battery is low, go to low battery mode
        batt_percent = self.parent.telemetry.gom.percent
        if (batt_percent < params.ENTER_LOW_BATTERY_MODE_THRESHOLD) \
                and not params.IGNORE_LOW_BATTERY:
            return FMEnum.LowBatterySafety.value

        # if there is no current coming into the batteries, go to low battery mode
        if sum(self.parent.telemetry.gom.hk.curin) < params.ENTER_ECLIPSE_MODE_CURRENT \
                and batt_percent < params.ENTER_ECLIPSE_MODE_THRESHOLD \
                and not params.IGNORE_LOW_BATTERY:
            return FMEnum.LowBatterySafety.value

        # go to comms mode if there is something in the comms queue
        if not self.parent.downlink_queue.empty():
            return FMEnum.CommsMode.value

        return NO_FM_CHANGE  # returns -1 if the logic here does not make any FM changes

    def register_commands(cls):
        raise NotImplementedError("Only implemented in specific flight mode subclasses")

    def run_mode(self):
        raise NotImplementedError("Only implemented in specific flight mode subclasses")

    def execute_commands(self):
        bogus = bool()
        if len(self.parent.commands_to_execute) == 0:
            pass  # If I have no commands to execute do nothing
        else:
            # loop through commands in commands_to_execute list
            finished_commands = []

            for command in self.parent.commands_to_execute:

                bogus = False
                try:
                    mac, counter, command_fm, command_id, command_kwargs = self.parent.command_handler.unpack_command(
                        command)
                    logger.info(f"Received command {command_fm}:{command_id} with args {str(command_kwargs)}")
                    assert command_fm in self.parent.command_definitions.COMMAND_DICT
                    assert command_id in self.parent.command_definitions.COMMAND_DICT[command_fm]
                except AssertionError:
                    logger.warning(f"Rejecting bogus command {command_fm}:{command_id}:{command_kwargs}")
                    bogus = True

                if bogus is not True:
                    # changes the flight mode if command's FM is different.
                    if command_fm != self.flight_mode_id:
                        self.parent.replace_flight_mode_by_id(command_fm)

                    # locate which method to run:
                    method_to_run = self.parent.command_definitions.COMMAND_DICT[command_fm][command_id]
                    method_to_run(**command_kwargs)  # run that method
                finished_commands.append(command)

                # Prioritize downlinking: execute all necessary downlinks before
                # Starting next command
                # TODO: Execute downlinks before moving on to next command

            # TODO: Add try/except/finally statement above so that the for loop below always runs, even if an
            #  exception occurs in the above for loop
            for finished_command in finished_commands:
                self.parent.commands_to_execute.remove(finished_command)

    def poll_inputs(self):

        if self.parent.gom is not None:
            self.parent.gom.tick_wdt()
        self.parent.telemetry.poll()

    def completed_task(self):
        self.task_completed = True

    # Autonomous actions to maintain safety
    def automatic_actions(self):
        pass

    def write_telemetry(self):
        pass

    def __enter__(self):
        logger.debug(f"Starting flight mode {self.flight_mode_id}")
        return self

    def __exit__(self, exc_type, exc_value, tb):
        logger.debug(f"Finishing flight mode {self.flight_mode_id}")
        if exc_type is not None:
            logger.error(f"Flight Mode failed with error type {exc_type} and value {exc_value}")
            logger.error(f"Failed with traceback:\n {tb}")


# Model for FlightModes that require precise timing
# Pause garbage collection and anything else that could
# interrupt critical thread
class PauseBackgroundMode(FlightMode):
    def register_commands(self):
        super().register_commands()

    def run_mode(self):
        super().run_mode()

    def __init__(self, parent):
        super().__init__(parent)

    def __enter__(self):
        super().__enter__()
        self.parent.nemo_manager.pause()
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()
        self.parent.nemo_manager.resume()
        super().__exit__(exc_type, exc_val, exc_tb)


class TestMode(PauseBackgroundMode):
    flight_mode_id = FMEnum.TestMode.value

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self):
        pass  # this is intentional - we don't want the FM to update if we are testing something

    def run_mode(self):
        pass

    command_codecs = {TestCommandEnum.SeparationTest.value: ([], 0),
                      TestCommandEnum.ADCTest.value: ([], 0),
                      TestCommandEnum.CommsDriver.value: ([], 0),
                      TestCommandEnum.PiShutdown.value: ([], 0),
                      TestCommandEnum.RTCTest.value: ([], 0),
                      TestCommandEnum.LongString.value: (['some_number', 'long_string'],180)
                      }

    command_arg_types = {
        'some_number': 'float',
        'long_string':'string'
    }

    downlink_codecs = {TestCommandEnum.CommsDriver.value: (['gyro1', 'gyro2', 'gyro3'], 12)}

    downlink_arg_unpackers = {
        'gyro1': 'float',
        'gyro2': 'float',
        'gyro3': 'float',
    }


class CommsMode(FlightMode):
    flight_mode_id = FMEnum.CommsMode.value

    def __init__(self, parent):
        super().__init__(parent)
        self.electrolyzing = False

    def enter_transmit_safe_mode(self):

        # Stop electrolyzing
        if self.parent.gom.is_electrolyzing():
            self.electrolyzing = True
            self.parent.gom.set_electrolysis(False)

        # Set RF receiving side to low
        self.parent.gom.rf_receiving_switch(receive=False)

        # Turn off LNA
        self.parent.gom.lna(False)

        # Set RF transmitting side to high
        self.parent.gom.rf_transmitting_switch(receive=False)

        # Turn on power amplifier
        self.parent.gom.set_PA(on=True)

    def exit_transmit_safe_mode(self):

        # Turn off power amplifier
        self.parent.gom.set_PA(on=False)

        # Set RF transmitting side to low
        self.parent.gom.rf_transmitting_switch(receive=True)

        # Turn on LNA
        self.parent.gom.lna(True)

        # Set RF receiving side to high
        self.parent.gom.rf_receiving_switch(receive=True)

        # Resume electrolysis if we paused it to transmit
        if self.electrolyzing:
            self.parent.gom.set_electrolysis(True, delay=params.DEFAULT_ELECTROLYSIS_DELAY)

    def execute_downlinks(self):
        while not self.parent.downlink_queue.empty():
            self.parent.radio.transmit(self.parent.downlink_queue.get())
            self.parent.downlink_counter += 1
            sleep(params.DOWNLINK_BUFFER_TIME)

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm

    def run_mode(self):
        if not self.parent.downlink_queue.empty():
            self.enter_transmit_safe_mode()
            self.execute_downlinks()
            self.exit_transmit_safe_mode()

        self.completed_task()


class OpNavMode(FlightMode):
    flight_mode_id = FMEnum.OpNav.value

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        # TODO: overhaul so opnav calculation only occur while in opnav mode
        if not self.parent.opnav_process.is_alive():
            logger.info("[OPNAV]: Able to run next opnav")
            self.parent.last_opnav_run = datetime.now()
            logger.info("[OPNAV]: Starting opnav subprocess")
            self.parent.opnav_process = Process(target=self.opnav_subprocess, args=(self.parent.opnav_proc_queue,))
            self.parent.opnav_process.start()
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
    flight_mode_id = FMEnum.SensorMode.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError

    def update_state(self):
        return NO_FM_CHANGE  # intentional: we don't want to update FM when testing sensors


class LowBatterySafetyMode(FlightMode):
    flight_mode_id = FMEnum.LowBatterySafety.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError

    def run_mode(self):
        sleep(params.LOW_BATT_MODE_SLEEP)  # saves battery, maybe?
        raise NotImplementedError

    def update_state(self):
        # check power supply to see if I can transition back to NormalMode
        if self.parent.telemetry.gom.percent > params.EXIT_LOW_BATTERY_MODE_THRESHOLD:
            return FMEnum.Normal.value

    def __enter__(self):
        super().__enter__()
        self.parent.gom.all_off()  # turns everything off immediately upon entering mode to preserve power
        self.parent.gom.pc.set_GPIO_low()


class ManeuverMode(PauseBackgroundMode):
    flight_mode_id = FMEnum.Maneuver.value
    command_codecs = {}
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
        logger.info("Glowplug maneuver...")
        # TODO: poll and check accelerometer values. If not acceleration seen, try other glowplug
        self.parent.gom.glowplug(GLOWPLUG_DURATION)
        self.parent.maneuver_queue.get()
        self.task_completed = True


# TODO
class SafeMode(FlightMode):
    flight_mode_id = FMEnum.Safety.value

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        logger.info("Execute safe mode")


class NormalMode(FlightMode):

    def register_commands(cls):
        pass

    flight_mode_id = FMEnum.Normal.value

    command_codecs = {
        NormalCommandEnum.Switch.value: ([], 0),
        NormalCommandEnum.RunOpNav.value: ([], 0),
        NormalCommandEnum.SetDesiredAttitude.value: ([AZIMUTH, ELEVATION], 8),
        # NormalCommandEnum.SetAccelerate.value: ([ACCELERATE], 1),
        # NormalCommandEnum.SetBreakpoint.value: ([], 0),  # TODO define exact parameters
        NormalCommandEnum.SetParam.value: ([NAME, VALUE, HARD_SET], 33),
        NormalCommandEnum.SetElectrolysis.value: ([STATE, DELAY], 5),
        NormalCommandEnum.SetOpnavInterval.value: ([INTERVAL], 4),
        NormalCommandEnum.Verification.value: ([NUM_BLOCKS], 2),
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
        NormalCommandEnum.NemoPowerOff.value: ([], 0),
        NormalCommandEnum.NemoPowerOn.value: ([], 0),
        NormalCommandEnum.NemoReboot.value: ([], 0),
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
        NormalCommandEnum.ExecPyFile.value: ([FNAME], 36)
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
        CMD: 'string',
        FNAME: 'string'
    }

    downlink_codecs = {
        NormalCommandEnum.BasicTelem.value: ([RTC_TIME, ATT_1, ATT_2, ATT_3, ATT_4,
                                              HK_TEMP_1, HK_TEMP_2, HK_TEMP_3, HK_TEMP_4, GYRO_TEMP, THERMOCOUPLER_TEMP,
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
        THERMOCOUPLER_TEMP: 'float',
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
        RETURN_CODE: "uint8"
    }

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self):
        super_fm = super().update_state()

        if super_fm != NO_FM_CHANGE:
            return super_fm

        time_for_opnav = (time() - self.parent.telemetry.opn.poll_time) // 60 < params.OPNAV_INTERVAL
        need_to_electrolyze = self.parent.telemetry.prs.pressure < params.IDEAL_CRACKING_PRESSURE
        currently_electrolyzing = self.parent.telemetry.gom.is_electrolyzing

        # if we don't want to electrolyze (per GS command), set need_to_electrolyze to false
        need_to_electrolyze = need_to_electrolyze and params.WANT_TO_ELECTROLYZE

        # if currently electrolyzing and over pressure, stop electrolyzing
        if currently_electrolyzing and not need_to_electrolyze:
            self.parent.gom.set_electrolysis(False)

        if currently_electrolyzing and need_to_electrolyze:
            pass  # we are already in the state we want to be in

        # if below pressure and not electrolyzing, start electrolyzing
        if not currently_electrolyzing and need_to_electrolyze:
            self.parent.gom.set_electrolysis(True)

        if not currently_electrolyzing and not need_to_electrolyze:
            pass  # we are already in the state we want to be in

        # note: at this point, the variable "need_to_electrolyze" is equivalent to the new state of the electrolyzer

        if time_for_opnav:
            logger.info("Time to run Opnav")
            return FMEnum.OpNav.value

        # if we have data to downlink, change to comms mode
        if not (self.parent.downlink_queue.empty()):
            return FMEnum.CommsMode.value

    def run_mode(self):
        logger.info(f"In NORMAL flight mode")
        self.completed_task()


class CommandMode(PauseBackgroundMode):
    """Command Mode: a Flight Mode that listens for and runs commands and nothing else."""

    flight_mode_id = FMEnum.Command.value


    command_codecs = {
        CommandCommandEnum.AddFileBlock.value:([FILE_PATH,BLOCK_NUMBER,BLOCK_TEXT],195 - MIN_COMMAND_SIZE),
        CommandCommandEnum.GetFileBlocksInfo.value: ([FILE_PATH, TOTAL_BLOCKS], 52),
        CommandCommandEnum.ActivateFile.value:([FILE_PATH,TOTAL_BLOCKS],52)
        }

    command_arg_types = {
        FILE_PATH: 'string',
        BLOCK_NUMBER: 'short',
        BLOCK_TEXT: 'string',
        TOTAL_BLOCKS: 'short'
    }

    downlink_codecs = {
        CommandCommandEnum.AddFileBlock.value: ([SUCCESSFUL,BLOCK_NUMBER],3),
        CommandCommandEnum.GetFileBlocksInfo.value: ([CHECKSUM, MISSING_BLOCKS], 80 - MIN_COMMAND_SIZE)
    }

    downlink_arg_types = {
        SUCCESSFUL: 'bool',
        BLOCK_NUMBER: 'short',
        CHECKSUM: 'string',
        MISSING_BLOCKS: 'string'
    }

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self):
        # DO NOT TICK THE WDT
        return NO_FM_CHANGE  # intentional

    def run_mode(self):
        pass  # intentional

    def poll_inputs(self):
        # TODO
        raise NotImplementedError  # only check the comms queue
