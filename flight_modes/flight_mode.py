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
import os

from utils.constants import (  # noqa F401
    LOW_CRACKING_PRESSURE,
    EXIT_LOW_BATTERY_MODE_THRESHOLD,
    HIGH_CRACKING_PRESSURE,
    IDEAL_CRACKING_PRESSURE,
    OPNAV_INTERVAL,
    BOOTUP_SEPARATION_DELAY,
    FMEnum,
    NormalCommandEnum,
    BootCommandEnum,
    TestCommandEnum,
    ManeuverCommandEnum,
    POSITION_X,
    POSITION_Y,
    POSITION_Z,
    ACCELERATE,
    NAME,
    VALUE,
    AZIMUTH,
    ELEVATION,
    STATE,
    INTERVAL,
    DELAY,
    NO_FM_CHANGE,
    GLOWPLUG_DURATION,
    BURN_WAIT_TIME
)

from utils.log import get_log

from utils.exceptions import UnknownFlightModeException
from utils.struct import (
    pack_bool,
    pack_unsigned_short,
    pack_double,
    pack_unsigned_int,
    pack_str,
    pack_float,
    unpack_bool,
    unpack_unsigned_short,
    unpack_double,
    unpack_unsigned_int,
    unpack_str,
    unpack_float
)



# Necessary modes to implement
# BootUp, Restart, Normal, Eclipse, Safety, Propulsion,
# Attitude Adjustment, Transmitting, OpNav (image processing)
# TestModes

no_transition_modes = [
    FMEnum.SensorMode.value,
    FMEnum.TestMode.value,
    FMEnum.CommsMode.value,
    FMEnum.Command.value
]

logger = get_log()


class FlightMode:
    # Override in Subclasses to tell CommandHandler the functions and arguments this flight mode takes
    command_codecs = {}

    # Map argument names to (packer,unpacker) tuples
    # This tells CommandHandler how to serialize the arguments for commands to this flight mode
    command_arg_unpackers = {}

    flight_mode_id = -1  # Value overridden in FM's implementation

    def __init__(self, parent: MainSatelliteThread):
        self.parent = parent
        self.task_completed = False
        self.burn_time = None

    def update_state(self) -> int:
        """update_state returns the id of the flight mode that we want to change to, which is then used in main.py's
        update_state to update our flight mode """
        # currently a mess and needs revisiting. Formal logic for switching FMs has not been defined/documented.
        # Please do so!
        flight_mode_id = self.flight_mode_id

        if self.task_completed and not self.parent.FMQueue.empty():
            return self.parent.FMQueue.get()

        if flight_mode_id in no_transition_modes:
            return flight_mode_id

        # go to maneuver mode
        if not self.parent.maneuver_queue.empty():
            # TODO assign the time command to this var
            self.burn_time = None
            if self.burn_time - time() < (60.0*BURN_WAIT_TIME):
                return FMEnum.Maneuver.value

        # if battery is low, go to low battery mode
        batt_percent = self.parent.tlm.gom.percent
        if (batt_percent < self.parent.constants.ENTER_LOW_BATTERY_MODE_THRESHOLD) \
                and not self.parent.constants.IGNORE_LOW_BATTERY:
            return FMEnum.LowBatterySafety.value

        # if there is no current coming into the batteries, go to low battery mode
        if sum(self.parent.tlm.gom.curin) < self.parent.constants.ENTER_ECLIPSE_MODE_CURRENT \
                and batt_percent < self.parent.constants.ENTER_ECLIPSE_MODE_THRESHOLD \
                and not self.parent.constants.IGNORE_LOW_BATTERY:
            return FMEnum.LowBatterySafety.value

        # go to comms mode
        if not self.parent.communications_queue.empty():
            return FMEnum.CommsMode.value

        return NO_FM_CHANGE  # returns -1 if the logic here does not make any FM changes

        # everything in update_state below this comment should be implemented in their respective flight mode
        # The logic defined below isn't necessarily incorrect - it's just in an outdated format

        # Burn command queue logic
        # TODO implment need_to_burn function in adc driver
        if self.parent.pressure_sensor.need_to_burn():
            self.parent.replace_flight_mode_by_id(FMEnum.Maneuver.value)
            return

        # Check if opnav needs to be run
        curr_time = datetime.now()
        time_diff = curr_time - self.parent.last_opnav_run
        if time_diff.seconds * 60 > OPNAV_INTERVAL:
            self.parent.replace_flight_mode_by_id(FMEnum.OpNav.value)

        elif flight_mode_id == FMEnum.LowBatterySafety.value:
            if (
                self.gom.read_battery_percentage()
                >= EXIT_LOW_BATTERY_MODE_THRESHOLD
            ):
                self.parent.replace_flight_mode_by_id(FMEnum.Normal.value)

        elif flight_mode_id == FMEnum.Safety.value:
            raise NotImplementedError  # TODO

        elif flight_mode_id == FMEnum.Maneuver.value:
            if self.task_completed is True:
                self.parent.replace_flight_mode_by_id(FMEnum.Normal.value)

        elif flight_mode_id == FMEnum.OpNav.value:
            if self.task_completed is True:
                self.parent.replace_flight_mode_by_id(FMEnum.Normal.value)

        elif flight_mode_id in no_transition_modes:
            pass

        else:
            raise UnknownFlightModeException(flight_mode_id)

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
                    command_fm, command_id, command_kwargs = self.parent.command_handler.unpack_command(command)
                    logger.info(f"Received command {command_fm}:{command_id} with args {str(command_kwargs)}")
                    assert command_fm in self.parent.command_definitions.COMMAND_DICT
                    assert command_id in self.parent.command_definitions.COMMAND_DICT[command_fm]
                except AssertionError:
                    self.parent.logger.warning(f"Rejecting bogus command {command_fm}:{command_id}:{command_kwargs}")
                    bogus = True

                if bogus is not True:
                    # changes the flight mode if command's FM is different.
                    if command_fm != self.flight_mode_id:
                        self.parent.replace_flight_mode_by_id(command_fm)

                    # locate which method to run:
                    method_to_run = self.parent.command_definitions.COMMAND_DICT[command_fm][command_id]
                    method_to_run(**command_kwargs)  # run that method
                finished_commands.append(command)
            # TODO: Add try/except/finally statement above so that the for loop below always runs, even if an
            #  exception occurs in the above for loop
            for finished_command in finished_commands:
                self.parent.commands_to_execute.remove(finished_command)

    def poll_inputs(self):
        self.parent.gom.tick_wdt()
        self.parent.tlm.poll()

    def completed_task(self):
        self.task_completed = True

    # Autonomous actions to maintain safety
    def automatic_actions(self):
        pass

    def write_telemetry(self):
        pass

    def __enter__(self):
        self.parent.logger.info(f"Starting flight mode {self.flight_mode_id}")
        return self

    def __exit__(self, exc_type, exc_value, tb):
        self.parent.logger.info(f"Finishing flight mode {self.flight_mode_id}")
        if exc_type is not None:
            self.parent.logger.error(f"Flight Mode failed with error type {exc_type} and value {exc_value}")
            self.parent.logger.error(f"Failed with traceback:\n {tb}")


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
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()
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
                      TestCommandEnum.ADCTest.value: ([], 0)}

    command_arg_unpackers = {}


class CommsMode(FlightMode):
    flight_mode_id = FMEnum.CommsMode.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError


class OpNavMode(FlightMode):
    """dummy FM for now"""
    flight_mode_id = FMEnum.OpNav.value

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        from core.opnav import start
        start()

    def update_state(self) -> int:
        super_fm = super().update_state()
        if super_fm != NO_FM_CHANGE:
            return super_fm

        # check if opnav db has been updated, then set self.task_completed true
        if self.task_completed:
            return FMEnum.Normal.value

        return NO_FM_CHANGE


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

    # TODO point solar panels directly at the sun

    def run_mode(self):
        sleep(self.parent.constants.LOW_BATT_MODE_SLEEP)  # saves battery, maybe?
        raise NotImplementedError

    def update_state(self):
        # check power supply to see if I can transition back to NormalMode
        if self.parent.tlm.gom.percent > self.parent.constants.EXIT_LOW_BATTERY_MODE_THRESHOLD:
            self.parent.replace_flight_mode_by_id(self.parent.constants.FMEnum.Normal.value)

        if sum(self.parent.tlm.gom.hk.curin) > self.parent.constants.ENTER_ECLIPSE_MODE_CURRENT:
            # If we do have some power coming in (i.e. we are not eclipsed by the moon/earth), reorient to face sun
            raise NotImplementedError

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
            return FMEnum.Normal.value
        return NO_FM_CHANGE

    def run_mode(self):
        # sleeping for 5 fewer seconds than the delay for safety
        sleep((self.burn_time - time()) - 5)
        self.parent.gom.glowplug(GLOWPLUG_DURATION)
        self.task_completed = True


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
        NormalCommandEnum.SetDesiredAttitude.value: (
            [AZIMUTH, ELEVATION], 8),
        # NormalCommandEnum.SetAccelerate.value: ([ACCELERATE], 1),
        # NormalCommandEnum.SetBreakpoint.value: ([], 0),  # TODO define exact parameters
        NormalCommandEnum.SetParam.value: ([NAME, VALUE], 12),
        NormalCommandEnum.SetElectrolysis.value: ([STATE, DELAY], 5),
        NormalCommandEnum.SetOpnavInterval.value: ([INTERVAL], 4)
    }

    command_arg_unpackers = {
        AZIMUTH: (pack_float, unpack_float),
        ELEVATION: (pack_float, unpack_float),
        ACCELERATE: (pack_bool, unpack_bool),
        NAME: (pack_str, unpack_str),
        # TODO: can't use strings in current configuration b/c command_codecs requires a fixed number of bytes
        VALUE: (pack_double, unpack_double),
        STATE: (pack_bool, unpack_bool),
        INTERVAL: (pack_unsigned_int, unpack_unsigned_int),
        DELAY: (pack_unsigned_short, unpack_unsigned_short)
    }

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self):
        super_fm = super().update_state()

        if super_fm != NO_FM_CHANGE:
            return super_fm

        time_for_opnav = (time() - self.parent.tlm.opn.poll_time) // 60 < self.parent.constants.OPNAV_INTERVAL
        need_to_electrolyze = self.parent.tlm.prs.pressure < self.parent.constants.IDEAL_CRACKING_PRESSURE
        currently_electrolyzing = self.parent.tlm.gom.is_electrolyzing
        seconds_to_electrolyze = 60  # TODO: actual calculation involving current pressure

        # if we don't want to electrolyze (per GS command), set need_to_electrolyze to false
        need_to_electrolyze = need_to_electrolyze and self.parent.constants.WANT_TO_ELECTROLYZE

        # There's probably some super-optimized branchless way of implementing this logic, but oh well:

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
            if need_to_electrolyze:
                # If it's time for opnav to run BUT we are below ideal pressure: turn off electrolysis after a delay
                self.parent.gom.set_electrolysis(False, delay=seconds_to_electrolyze)
                # may not be relevant anymore now that opnav is becoming a subprocess

            return FMEnum.OpNav.value

        # if we have data to downlink, change to comms mode
        if not (self.parent.communications_queue.empty()):
            return FMEnum.CommsMode.value

    def run_mode(self):
        logger.info(f"In NORMAL flight mode")


class CommandMode(PauseBackgroundMode):
    """Command Mode: a Flight Mode that listens for and runs commands and nothing else."""

    flight_mode_id = FMEnum.Command.value

    command_codecs = {}

    command_arg_unpackers = {}

    def __init__(self, parent):
        super().__init__(parent)

    def update_state(self):
        # DO NOT TICK THE WDT
        return NO_FM_CHANGE  # intentional

    def run_mode(self):
        pass  # intentional

    def poll_inputs(self):
        raise NotImplementedError  # only check the comms queue
