from enum import IntEnum, unique
import os


# TODO
# Create method to set constants based on purpose of run

# Delay to wait on BootUp
BOOTUP_SEPARATION_DELAY = 30.0


# TODO determine correct values threshold values
ENTER_LOW_BATTERY_MODE_THRESHOLD = 0.3
EXIT_LOW_BATTERY_MODE_THRESHOLD = 0.5


# Constants defining goal cracking pressures for electrolysis
LOW_CRACKING_PRESSURE = 10.0
HIGH_CRACKING_PRESSURE = 20.0
IDEAL_CRACKING_PRESSURE = 15.0


CISLUNAR_BASE_DIR = os.path.join(os.path.expanduser("~"), ".cislunar-flight-software")  # noqa E501
LOG_DIR = os.path.join(CISLUNAR_BASE_DIR, "logs")
DB_FILE = os.path.join(CISLUNAR_BASE_DIR, "db.sqlite")


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
