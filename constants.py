from enum import IntEnum
import os

# Create method to set constants based on purpose of running the code ie. test vs flight values

# Delay to wait on BootUp
BOOTUP_SEPARATION_DELAY = 3.0  # TODO change to 30.0 for flight


# Constants defining goal cracking pressures for electrolysis
LOW_CRACKING_PRESSURE = 10.0
HIGH_CRACKING_PRESSURE = 20.0
IDEAL_CRACKING_PRESSURE = 15.0


LOG_DIR = os.path.join(os.path.expanduser("~"), ".cislunar-flight-software", "logs")


@unique
class FlightModeEnum(IntEnum):
    Boot = 0
    Restart = 1
    Normal = 2
    LowBatterySafety = 3
    Safety = 4
    OpNav = 5
    Electrolysis = 6
    Maneuver = 7
