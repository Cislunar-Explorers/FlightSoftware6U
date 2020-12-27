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

# OpNav timing interval in minutes
OPNAV_INTERVAL = 60

SQL_PREFIX = "sqlite:///"
CISLUNAR_BASE_DIR = os.path.join(
    os.path.expanduser("~"), ".cislunar-flight-software"
)
LOG_DIR = os.path.join(CISLUNAR_BASE_DIR, "logs")
DB_FILE = SQL_PREFIX + os.path.join(CISLUNAR_BASE_DIR, "satellite-db.sqlite")


MODE_SIZE = 1
ID_SIZE = 1
DATA_LEN_SIZE = 2
MIN_COMMAND_SIZE = MODE_SIZE + ID_SIZE + DATA_LEN_SIZE

MODE_OFFSET = 0
ID_OFFSET = 1
DATA_LEN_OFFSET = 2
DATA_OFFSET = 4


# Keyword Argument Definitions for Commands
POSITION_X = "position_x"
POSITION_Y = "position_y"
POSITION_Z = "position_z"

ACCELERATE = "accelerate"

ATTITUDE_X = "attitude_x"
ATTITUDE_Y = "attitude_y"
ATTITUDE_Z = "attitude_z"


# GOMspace Channel designations:
class GomOutputs(IntEnum):
    comms = 0
    burnwire_1 = 1
    burnwire_2 = 2
    glowplug = 3
    solenoid = 4
    electrolyzer = 5


GOM_VOLTAGE_MAX = 8400
GOM_VOLTAGE_MIN = 6000


@unique
class FMEnum(IntEnum):
    Boot = 0
    Restart = 1
    Normal = 2
    LowBattery = 3
    Safety = 4
    OpNav = 5
    Maneuver = 6
    SensorMode = 7  # Send command directly to sensor
    TestMode = 8  # Execute specified test
    CommsMode = 9
    OpNavManeuver = 10


@unique
class BootCommandEnum(IntEnum):
    Separate = 0


@unique
class RestartCommandEnum(IntEnum):
    pass


@unique
class NormalCommandEnum(IntEnum):
    RunOpNav = 0  # no args
    SetDesiredAttitude = 1  # arg=attitude
    SetAccelerate = 2  # arg=true/false
    SetBreakpoint = 3  # arg=position x, y, z


@unique
class LowBatterySafetyCommandEnum(IntEnum):
    ExitLBSafetyMode = 0  # no args, # XXX this is an override command
    SetExitLBSafetyMode = 1  # define battery percentage


@unique
class SafetyCommandEnum(IntEnum):
    ExitSafetyMode = 0
    SetExitSafetyMode = 1


@unique
class OpNavCommandEnum(IntEnum):
    RunOpNav = 0  # no args
    SetInterval = 1  # arg=interval in minutes packed as an int


@unique
class ManeuverCommandEnum(IntEnum):
    RunOpNav = 0  # no args
    SetDesiredAttitude = 1  # arg=attitude
    SetAccelerate = 2  # arg=true/false
    SetBreakpoint = 3  # arg=?


@unique
class SensorsCommandEnum(IntEnum):
    Thermocouple = 0
    PressureTransducer = 1
    Gomspace = 2
    CameraMux = 3
    Gyro = 4
    RTC = 5
    AX5043 = 6


@unique
class TestCommandEnum(IntEnum):
    SetTestMode = 0  # no args
    TriggerBurnWire = 1  # no args
    RunOpNav = 2  # no args


@unique
class CommsCommandEnum(IntEnum):
    DownlinkFullDataPacket = 4  # no args
    SetDataPacket = 5  # arg=data packet id


