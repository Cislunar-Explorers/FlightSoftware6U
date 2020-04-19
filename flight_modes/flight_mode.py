import gc
from time import sleep
import os

from utils.constants import (  # noqa F401
    LOW_CRACKING_PRESSURE,
    HIGH_CRACKING_PRESSURE,
    IDEAL_CRACKING_PRESSURE,
    BOOTUP_SEPARATION_DELAY,
    FlightModeEnum,
)

# Necessary modes to implement
# BootUp, Restart, Normal, Eclipse, Safety, Electrolysis, Propulsion,
# Attitude Adjustment, Transmitting, OpNav (image processing)
# TestModes


class FlightMode:
    def __init__(self, parent):
        self.parent = parent
        self.task_completed = False

    def run_mode(self):
        pass

    def read_sensors(self):
        pass

    def completed_task(self):
        self.task_completed = True

    # Autonomous actions to maintain safety
    def automatic_actions(self):
        pass

    def write_telemetry(self):
        pass


# BootUp mode tasks:
# Sleep for 30 seconds to reach safe distance from Artemis I
class BootUpMode(FlightMode):

    flight_mode_id = FlightModeEnum.Boot.value

    def __init__(self, parent):
        super().__init__(parent)
        self.set_started_bootup()
        print("Booting up...")

    def set_started_bootup(self):
        # TODO implement logger to actually put logs in this directory
        os.makedirs(self.parent.log_dir)

    # Sleep for a delay set by BOOTUP_SEPARATION_DELAY if I haven't done so already
    # If I've already completed the delay, do nothing and await separation and
    # first burn command from ground station
    def run_mode(self):
        if not self.delay_completed:
            sleep(BOOTUP_SEPARATION_DELAY)
            self.task_completed()


class RestartMode(FlightMode):

    flight_mode_id = FlightModeEnum.Restart.value

    def __init__(self, parent):
        super().__init__(parent)
        self.clear_unnecessary_storage_and_memory()
        self.check_sensors()
        self.completed_task()
        # TODO
        # trigger restart again if necessary
        # make note of turning back on and immediately downlink that we've restarted
        # increment counter for consecutive restarts in db

    def clear_unnecessary_storage_and_memory(self):
        gc.collect()
        # TODO clean up storage if necessary
        # MAKE SURE THIS ONLY HAPPENS ON THE RPI!!!

    # TODO ensure sensors are working correctly
    def check_sensors(self):
        pass

    # Since restart is handled in initialization of this flight mode, it should always
    # transition to NormalMode before running mode for restart
    def run_mode(self):
        pass


# Electrolyze until pressure in the tank reaches IDEAL_CRACKING_PRESSURE
# TODO determine correct values based on testing
# TODO determine if we should change to electrolyze until going above LOW_CRACKING_PRESSURE
# NOTE: if we go with LOW_CRACKING_PRESSURE ensure that pressure doesn't dip below in interval between exiting
# electrolysis and hitting ignition
class ElectrolysisMode(FlightMode):

    flight_mode_id = FlightModeEnum.Electrolysis.value

    def __init__(self, parent):
        super().__init__(parent)
        self.pressure_sensor = parent.pressure_sensor
        self.gom = parent.gom

    # Turn electrolysis on if propellant tank is below IDEAL_CRACKING_PRESSURE
    # If we have reached this cracking pressure, then turn electrolysis off
    # and set task_completed to True
    def run_mode(self):
        # If electrolyzing is turned on, check to see if I should turn it off
        curr_pressure = self.pressure_sensor.read_value()
        if curr_pressure >= IDEAL_CRACKING_PRESSURE:
            self.gom.set_electrolysis(False)
            self.completed_task()
        else:
            self.gom.set_electrolysis(True)


class LowBatterySafetyMode(FlightMode):

    flight_mode_id = FlightModeEnum.LowBatterySafety.value

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
    def __init__(self, parent):
        super().__init__(parent)

    def __enter__(self):
        gc.disable()

    def __exit__(self, exc_type, exc_val, exc_tb):
        gc.collect()
        gc.enable()


class ManeuverMode(PauseBackgroundMode):
    def __init__(self, parent, goal: int):
        super().__init__(parent)
        self.goal = goal
        self.moves_towards_goal = 0

    def execute_maneuver_towards_goal(self):
        self.moves_towards_goal += 1

    # TODO implement actual maneuver execution
    # check if exit condition has completed
    def run_mode(self):
        self.moves_towards_goal()
        if self.moves_towards_goal >= self.goal:
            self.task_completed()


class SafeMode(FlightMode):
    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Execute safe mode")


class NormalMode(FlightMode):
    def __init__(self, parent):
        super().__init__(self, parent)

    def run_mode(self):
        print("Execute normal mode")
