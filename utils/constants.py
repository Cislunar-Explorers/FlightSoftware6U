from enum import IntEnum, unique
import os


# TODO
# Create method to set constants based on purpose of run

# Delay to wait on BootUp
BOOTUP_SEPARATION_DELAY = 30.0

# TODO determine correct values threshold values
ENTER_LOW_BATTERY_MODE_THRESHOLD = 0.3
EXIT_LOW_BATTERY_MODE_THRESHOLD = 0.5

LOW_BATT_MODE_SLEEP = 10

ENTER_ECLIPSE_MODE_THRESHOLD = 0.5
EXIT_ECLIPSE_MODE_THRESHOLD = 0.75

ENTER_ECLIPSE_MODE_CURRENT = 50  # mA

# Constants defining goal cracking pressures for electrolysis
LOW_CRACKING_PRESSURE = 10.0
HIGH_CRACKING_PRESSURE = 20.0
IDEAL_CRACKING_PRESSURE = 15.0

# Set behaviour of electrolysis:
WANT_TO_ELECTROLYZE = True

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

NAME = "name"
VALUE = "value"

AZIMUTH = "theta"
ELEVATION = "phi"

STATE = "state"
INTERVAL = "interval"
DELAY = "delay"

GOM_VOLTAGE_MAX = 8400  # mV
GOM_VOLTAGE_MIN = 6000

# TODO: validate these values:
SPLIT_BURNWIRE_DURATION = 1  # second
ANTENNAE_BURNWIRE_DURATION = 1  # second
GLOWPLUG_DURATION = 1  # SECOND

MAX_GYRO_RATE = 250  # degrees/sec


@unique
class ConstantsEnum(IntEnum):
    GOM_VOLTAGE_MAX = 1


# GOMspace Channel designations:
# TODO: re-evaluate and double check before flight for each satellite half
@unique
class GomOutputs(IntEnum):
    comms = 0
    burnwire_1 = 1
    burnwire_2 = 2
    glowplug = 3
    solenoid = 4
    electrolyzer = 5


@unique
class FMEnum(IntEnum):
    Boot = 0
    Restart = 1
    Normal = 2
    LowBatterySafety = 3
    Safety = 4
    OpNav = 5
    Maneuver = 6
    SensorMode = 7  # Send command directly to sensor
    TestMode = 8  # Execute specified test
    CommsMode = 9
    Command = 10


@unique
class BootCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    Split = 1


@unique
class RestartCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands


@unique
class NormalCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    RunOpNav = 1  # no args
    SetDesiredAttitude = 2  # arg=attitude # i think this should only be allowed in maneuver mode
    SetElectrolysis = 3  # arg = bool whether to start or stop electrolysis
    # Really not sure what 3 and 4 are supposed to do:
    # SetAccelerate = 3  # arg=true/false
    # SetBreakpoint = 4  # arg=position x, y, z
    SetParam = 5
    CritTelem = 6
    BasicTelem = 7
    DetailedTelem = 8
    Verification = 9
    GetParam = 11
    SetOpnavInterval = 12



@unique
class LowBatterySafetyCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    ExitLBSafetyMode = 1  # no args, # XXX this is an override command
    SetExitLBSafetyMode = 2  # define battery percentage
    SetParam = 5
    CritTelem = 6
    BasicTelem = 7
    DetailedTelem = 8


@unique
class SafetyCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    ExitSafetyMode = 1
    SetExitSafetyMode = 2
    SetParameter = 5
    CritTelem = 6
    BasicTelem = 7
    DetailedTelem = 8


@unique
class OpNavCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    RunOpNav = 1  # no args
    SetInterval = 2  # arg=interval in minutes packed as an int


@unique
class ManeuverCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    RunOpNav = 1  # no args
    SetDesiredAttitude = 2  # arg=attitude
    SetAccelerate = 3  # arg=true/false
    SetBreakpoint = 4  # arg=?
    SetBurnTime = 9  # 1 arg: time at which thruster fires


@unique
class SensorsCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    Thermocouple = 1
    PressureTransducer = 2
    Gomspace = 3
    CameraMux = 4
    Gyro = 5
    RTC = 6
    AX5043 = 7


@unique
class TestCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    SetTestMode = 1  # no args
    TriggerBurnWire = 2  # no args
    RunOpNav = 3  # no args
    ADCTest = 4
    SeparationTest = 5
    GomPin = 6


@unique
class CommsCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    DownlinkFullDataPacket = 4  # no args
    SetDataPacket = 5  # arg=data packet id


@unique
class CommandCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    SetParam = 1  # 2 args: key and value of parameter to be changed
    SetSystemTime = 2  # 1 arg: UTC(?) time that the system clock should be set to
    RebootPi = 3
    RebootGom = 4
    PowerCycle = 5
    GomPin = 6  # 1 arg: which gom pin to toggle
    GomGeneralCmd = 7
    GeneralCmd = 8
    CeaseComms = 170
