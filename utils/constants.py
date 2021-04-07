from enum import IntEnum, unique
import os
import hashlib

# unit conversions

DEG2RAD = 3.14159265359 / 180

# Delay to wait on BootUp
# TODO change back to 30.0
BOOTUP_SEPARATION_DELAY = 5.0

# Verification Key Parameters
MAC_LENGTH = 4
MAC_DATA = b'Hello'  # FIXME for flight
MAC_KEY = b'World'
MAC = hashlib.blake2s(MAC_DATA, digest_size=MAC_LENGTH, key=MAC_KEY).digest()

# Serialization Sizes
MODE_SIZE = 1
ID_SIZE = 1
COUNTER_SIZE = 3
DATA_LEN_SIZE = 2
MIN_COMMAND_SIZE = MAC_LENGTH + COUNTER_SIZE + MODE_SIZE + ID_SIZE + DATA_LEN_SIZE

# Serializations Offsets
MAC_OFFSET = 0
COUNTER_OFFSET = 0 + MAC_LENGTH
MODE_OFFSET = COUNTER_SIZE + MAC_LENGTH
ID_OFFSET = 1 + COUNTER_SIZE + MAC_LENGTH
DATA_LEN_OFFSET = 2 + COUNTER_SIZE + MAC_LENGTH
DATA_OFFSET = 4 + COUNTER_SIZE + MAC_LENGTH

# Important paths
FLIGHT_SOFTWARE_PATH = '/home/pi/FlightSoftware/'
PARAMETERS_JSON_PATH = FLIGHT_SOFTWARE_PATH + 'utils/parameters.json'

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

START = "pulse_start"
PULSE_DURATION = "pulse_duration"
PULSE_NUM = "pulse_num"
PULSE_DT = "pulse_dt"

NUM_BLOCKS = "num_blocks"

TIME = "time"

HARD_SET = "hard_set"

FILE_PATH = "file_path"
BLOCK_NUMBER = "block_number"
BLOCK_TEXT = "block_text"
TOTAL_BLOCKS = "total_blocks"

# Keyword argument definitions for downlink
RTC_TIME = "rtc_time"

ATT_1 = "attitude_1"
ATT_2 = "attitude_2"
ATT_3 = "attitude_3"
ATT_4 = "attitude_4"

HK_TEMP_1 = "hk_temp_1"
HK_TEMP_2 = "hk_temp_2"
HK_TEMP_3 = "hk_temp_3"
HK_TEMP_4 = "hk_temp_4"
GYRO_TEMP = "gyro_temp"

THERMOCOUPLER_TEMP = "thermo_temp"

CURRENT_IN_1 = "curin_1"
CURRENT_IN_2 = "curin_2"
CURRENT_IN_3 = "curin_3"

VBOOST_1 = "vboost_1"
VBOOST_2 = "vboost_2"
VBOOST_3 = "vboost_3"

SYSTEM_CURRENT = "cursys"
BATTERY_VOLTAGE = "vbatt"

PROP_TANK_PRESSURE = "prs_pressure"

SUCCESSFUL = "successful"

MISSING_BLOCKS = "missing_blocks"
CHECKSUM = "checksum"

# SQL Stuff
SQL_PREFIX = "sqlite:///"
CISLUNAR_BASE_DIR = os.path.join(
    os.path.expanduser("~"), ".cislunar-flight-software"
)
LOG_DIR = os.path.join(CISLUNAR_BASE_DIR, "logs")
DB_FILE = SQL_PREFIX + os.path.join(CISLUNAR_BASE_DIR, "satellite-db.sqlite")

a = 1664525
b = 1013904223
M = 2 ** 32
team_identifier = 0xEB902D2D  # Team 2

# TODO: validate these values:
SPLIT_BURNWIRE_DURATION = 1  # second
ANTENNAE_BURNWIRE_DURATION = 1  # second
GLOWPLUG_DURATION = 1  # SECOND
BURN_WAIT_TIME = 15  # minutes

MAX_GYRO_RATE = 250  # degrees/sec # TODO

NO_FM_CHANGE = -1

GOM_TIMING_FUDGE_FACTOR = 3  # milliseconds

# Gyro specific constants
# TODO: make sure that we change this to 500 if need be
GYRO_RANGE = 250  # degrees per second


# GOMspace Channel designations:
# TODO: re-evaluate and double check before flight for each satellite half
@unique
class GomOutputs(IntEnum):
    comms = 0
    burnwire_1 = 1
    glowplug_2 = 2
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
    AttitudeAdjustment = 11


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
    WhenReorient = 13  # when we want to schedule a reorientation maneuver
                       # 2 args, unix time stamp and spin axis vector (2 floats)
    ScheduleReorientation = 14
    ScheduleManeuver = 15
    ACSPulsing = 16



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
    CommsDriver = 7
    RTCTest = 8
    LongString = 9
    PiShutdown = 11
    


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
    AddFileBlock = 9
    GetFileBlocksInfo = 10
    ActivateFile = 11
    CeaseComms = 170
