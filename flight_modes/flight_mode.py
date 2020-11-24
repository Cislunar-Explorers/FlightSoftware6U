import gc
from time import sleep
from datetime import datetime
import os
from main import MainSatelliteThread

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
    POSITION_X,
    POSITION_Y,
    POSITION_Z,
    ATTITUDE_X,
    ATTITUDE_Y,
    ATTITUDE_Z,
    ACCELERATE,
)
from utils.exceptions import UnknownFlightModeException
from utils.struct import (
    pack_bool,
    pack_double,
    unpack_bool,
    unpack_double,
)

from communications.command_definitions import CommandDefinitions

# Necessary modes to implement
# BootUp, Restart, Normal, Eclipse, Safety, Propulsion,
# Attitude Adjustment, Transmitting, OpNav (image processing)
# TestModes

no_transition_modes = [
    FMEnum.SensorMode.value,
    FMEnum.TestMode.value,
    FMEnum.CommsMode.value,
]


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
        self.commands = CommandDefinitions(parent)

    @classmethod
    def update_state(self):
        flight_mode_id = self.flight_mode_id

        # Burn command queue logic
        # TODO implment need_to_burn function in ADC driver
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
        if len(self.parent.commands_to_execute) == 0:
            pass  # If I have no commands to execute do nothing
        else:
            # loop through commands in commands_to_execute list
            for command in self.parent.commands_to_execute:
                command_fm, command_id, command_kwargs = self.parent.command_handler.unpack_command(command)
                # if command's FM is the same as the current FM, execute that command
                if command_fm == self.flight_mode_id:
                    method_to_run = self.commands[command_fm][command_id]
                    method_to_run(*command_kwargs)  # Will not work.
                    # This works assuming command_kwargs is actually a tuple of arguments (not a kwarg:value dictionary)
            # if the command's FM is different than the current FM, change FM and execute command in that FM

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
        self.parent.logger.info(f"Entering Flight Mode {self.flight_mode_id}")
        return self

    def __exit__(self, exc_type, exc_value, tb):
        self.parent.logger.info(f"Exiting Flight Mode {self.flight_mode_id}")
        if exc_type is not None:
            self.parent.logger.error(f"Flight Mode failed with error type {exc_type} and value {exc_value}")
            self.parent.logger.error(f"Failed with traceback:\n {tb}")


class TestMode(FlightMode):

    flight_mode_id = FMEnum.TestMode.value

    def __init__(self, parent):
        super().__init__(parent)
        raise NotImplementedError


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


# BootUp mode tasks:
# Sleep for 30 seconds to reach safe distance from Artemis I
class BootUpMode(FlightMode):
    command_codecs = {BootCommandEnum.Split.value: ([], 0)}
    command_arg_unpackers = {}

    flight_mode_id = FMEnum.Boot.value

    def __init__(self, parent):
        super().__init__(parent)
        self.set_started_bootup()
        print("Booting up...")

    def set_started_bootup(self):
        # TODO implement logger to actually put logs in this directory
        os.makedirs(self.parent.log_dir)

    # Sleep for a delay set by BOOTUP_SEPARATION_DELAY if I haven't
    # If I've already completed the delay, do nothing
    # Await separation command from ground station
    def run_mode(self):
        if not self.delay_completed:
            sleep(BOOTUP_SEPARATION_DELAY)
            self.task_completed()


class RestartMode(FlightMode):

    def register_commands(cls):
        pass

    command_codecs = {}
    command_arg_unpackers = {}
    flight_mode_id = FMEnum.Restart.value

    def __init__(self, parent):
        super().__init__(parent)
        self.clear_unnecessary_storage_and_memory()
        self.check_sensors()
        self.completed_task()
        # TODO
        # trigger restart again if necessary
        # make note of turning back on and immediately
        # # downlink restart information
        # increment counter for consecutive restarts in db

    def clear_unnecessary_storage_and_memory(self):
        gc.collect()
        # TODO clean up storage if necessary
        # pseudocode, if OS = raspian and environment = flight, not test:
        #   clean up storage and memory

    # TODO ensure sensors are working correctly
    def check_sensors(self):
        pass

    # Restart is handled in initialization
    # Therefore, should get updated to Normal before
    # ever running
    def run_mode(self):
        pass


class LowBatterySafetyMode(FlightMode):

    flight_mode_id = FMEnum.LowBatterySafety.value

    def __init__(self, parent):
        super().__init__(parent)

    # TODO point solar panels directly at the sun
    # check power supply to see if I can transition back to NormalMode
    def run_mode(self):
        pass


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
        super().__exit__()


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
        NormalCommandEnum.RunOpNav.value: ([], 0),
        NormalCommandEnum.SetDesiredAttitude.value: (
            [ATTITUDE_X, ATTITUDE_Y, ATTITUDE_Z],
            24,
        ),
        NormalCommandEnum.SetAccelerate.value: ([ACCELERATE], 1),
        # NormalCommandEnum.SetBreakpoint.value: ([], 0),  # TODO define exact parameters
    }

    command_arg_unpackers = {
        ATTITUDE_X: (pack_double, unpack_double),
        ATTITUDE_Y: (pack_double, unpack_double),
        ATTITUDE_Z: (pack_double, unpack_double),
        ACCELERATE: (pack_bool, unpack_bool),
    }

    def __init__(self, parent):
        super().__init__(parent)

    def run_mode(self):
        print("Execute normal mode")
