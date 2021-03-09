import gc
from time import sleep
from datetime import datetime
import os

"""from utils.constants import (  # noqa F401
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
    NUM_BLOCKS,
    RTC_TIME
)"""

from utils.constants import *
from utils.parameters import *
from utils.log import get_log

from utils.exceptions import UnknownFlightModeException

from communications.command_definitions import CommandDefinitions

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
    sensordata_codecs = {}
    downlink_codecs = {}

    # Map argument names to (packer,unpacker) tuples
    # This tells CommandHandler how to serialize the arguments for commands to this flight mode
    command_arg_types = {}
    sensordata_arg_unpackers = {}
    downlink_arg_types = {}

    flight_mode_id = -1  # Value overridden in FM's implementation

    def __init__(self, parent):
        self.parent = parent
        self.task_completed = False

    @classmethod
    def update_state(self):
        # currently a mess and needs revisiting. Formal logic for switching FMs has not been defined/documented.
        # Please do so!
        flight_mode_id = self.flight_mode_id

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

        elif flight_mode_id == FMEnum.Normal.value:
            pass
            # TODO do I need to enter electrolysis to prepare for maneuver?
            # do I need to start a maneuver?
            # do I need to run OpNav?

        elif flight_mode_id == FMEnum.Boot.value:
            pass

        elif flight_mode_id == FMEnum.Restart.value:
            if self.task_completed is True:
                self.parent.replace_flight_mode_by_id(FMEnum.Normal.value)

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
                    mac, counter, command_fm, command_id, command_kwargs = self.parent.command_handler.unpack_command(command)
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

                #Prioritize downlinking: execute all necessary downlinks before
                #Starting next command
                #TODO: Execute downlinks before moving on to next command

            # TODO: Add try/except/finally statement above so that the for loop below always runs, even if an
            #  exception occurs in the above for loop
            for finished_command in finished_commands:
                self.parent.commands_to_execute.remove(finished_command)

    def read_sensors(self):
        pass

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

    def run_mode(self):
        pass

    command_codecs = {TestCommandEnum.SeparationTest.value: ([], 0),
                      TestCommandEnum.ADCTest.value: ([], 0),
                      TestCommandEnum.CommsDriver.value:([],0),
                      TestCommandEnum.PiShutdown.value:([],0)
                      }

    command_arg_unpackers = {}

    downlink_codecs = {TestCommandEnum.CommsDriver.value:(['gyro1','gyro2','gyro3'],12)}

    downlink_arg_unpackers = {
        'gyro1': 'float',
        'gyro2': 'float',
        'gyro3': 'float',
        }


class CommsMode(FlightMode):
    flight_mode_id = FMEnum.CommsMode.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError


class SensorMode(FlightMode):

    flight_mode_id = FMEnum.SensorMode.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError

class LowBatterySafetyMode(FlightMode):

    flight_mode_id = FMEnum.LowBatterySafety.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError

    # TODO point solar panels directly at the sun
    # check power supply to see if I can transition back to NormalMode
    def run_mode(self):
        pass


class ManeuverMode(PauseBackgroundMode):
    flight_mode_id = FMEnum.Maneuver.value

    def __init__(self, parent):
        super().__init__(parent)
        self.goal = 10
        self.moves_towards_goal = 0

    def set_goal(self, goal: int):
        self.goal = 10

    def execute_maneuver_towards_goal(self):
        self.moves_towards_goal += 1

    # TODO implement actual maneuver execution
    # check if exit condition has completed
    def run_mode(self):
        self.moves_towards_goal()
        if self.moves_towards_goal >= self.goal:
            self.task_completed()


class SafeMode(FlightMode):

    flight_mode_id = FMEnum.Safety.value

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        print("Execute safe mode")


class NormalMode(FlightMode):

    def register_commands(cls):
        pass

    flight_mode_id = FMEnum.Normal.value

    command_codecs = {
        NormalCommandEnum.Switch.value: ([], 0),
        NormalCommandEnum.RunOpNav.value: ([], 0),
        NormalCommandEnum.SetDesiredAttitude.value: ([AZIMUTH, ELEVATION], 16),
        # NormalCommandEnum.SetAccelerate.value: ([ACCELERATE], 1),
        # NormalCommandEnum.SetBreakpoint.value: ([], 0),  # TODO define exact parameters
        NormalCommandEnum.SetParam.value: ([NAME, VALUE, HARD_SET], 33),
        NormalCommandEnum.SetElectrolysis.value: ([STATE, DELAY], 5),
        NormalCommandEnum.SetOpnavInterval.value: ([INTERVAL], 4),
        NormalCommandEnum.Verification.value: ([NUM_BLOCKS], 2)
    }

    command_arg_types = {
        AZIMUTH: 'double',
        ELEVATION: 'double',
        ACCELERATE: 'bool',
        NAME: 'string',
        # TODO: can't use strings in current configuration b/c command_codecs requires a fixed number of bytes
        VALUE: 'double',
        STATE: 'bool',
        INTERVAL: 'int',
        DELAY: 'short',
        NUM_BLOCKS: 'short',
        HARD_SET: 'bool'
    }

    downlink_codecs = {
        NormalCommandEnum.BasicTelem.value: ([RTC_TIME,ATT_1,ATT_2,ATT_3,ATT_4,
        HK_TEMP_1,HK_TEMP_2,HK_TEMP_3,HK_TEMP_4,GYRO_TEMP,THERMOCOUPLER_TEMP, CURRENT_IN_1,
        CURRENT_IN_2,CURRENT_IN_3,VBOOST_1,VBOOST_2,VBOOST_3,SYSTEM_CURRENT,BATTERY_VOLTAGE,
        PROP_TANK_PRESSURE], 216),

        NormalCommandEnum.SetParam.value:([SUCCESSFUL],1)
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
        HK_TEMP_1: 'float',
        HK_TEMP_2: 'float',
        HK_TEMP_3: 'float',
        HK_TEMP_4: 'float',
        GYRO_TEMP: 'float',
        THERMOCOUPLER_TEMP: 'float',
        CURRENT_IN_1: 'float',
        CURRENT_IN_2: 'float',
        CURRENT_IN_3: 'float',
        VBOOST_1: 'float',
        VBOOST_2: 'float',
        VBOOST_3: 'float',
        SYSTEM_CURRENT: 'float',
        BATTERY_VOLTAGE: 'float',
        PROP_TANK_PRESSURE: 'float',
        SUCCESSFUL: 'bool'
    }

    def __init__(self, parent):
        super().__init__(parent)
        self.last_opnav_run = self.parent.last_opnav_run

    def run_mode(self):
        time_since_last_run = datetime.now() - self.last_opnav_run
        minutes_since_last_run = time_since_last_run.total_seconds() / 60.0
        minutes_until_next_run = self.parent.parameters.OPNAV_INTERVAL - minutes_since_last_run
        if minutes_until_next_run < 0:
            CommandDefinitions(self.parent).run_opnav()
        logger.info(f"In NORMAL flight mode. Minutes until next OpNav run: {minutes_until_next_run}")