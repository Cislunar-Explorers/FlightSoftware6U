import os
from enum import IntEnum, unique
from dotenv import dotenv_values
from typing import cast

# dotenv vars
config = dotenv_values(".env")

CISLUNAR_BASE_DIR = cast("str", config["CISLUNAR_BASE_DIR"])
FLIGHT_SOFTWARE_PATH = cast("str", config["FLIGHT_SOFTWARE_PATH"])
FOR_FLIGHT = config["FOR_FLIGHT"] == "1"
LOG = config["LOG"] == "1"

# SQL Stuff
DB_ENTRY_LIMIT = 1000  # TODO update # maximum number of entries in any of the databases
DB_FILE = "sqlite:///" + os.path.join(CISLUNAR_BASE_DIR, "satellite-db.sqlite")
LOG_DIR = os.path.join(CISLUNAR_BASE_DIR, "logs")
NEMO_DIR = os.path.join(CISLUNAR_BASE_DIR, "nemo")

# Delay to wait on BootUp
# TODO change back to 30.0
BOOTUP_SEPARATION_DELAY = 5.0  # seconds

# Verification Key Parameters
MAC_LENGTH = 4
# MAC_DATA = b'Hello'
MAC_KEY = b"World"  # FIXME for flight
# MAC = hashlib.blake2s(MAC_DATA, digest_size=MAC_LENGTH, key=MAC_KEY).digest()

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
# FLIGHT_SOFTWARE_PATH = '/home/pi/FlightSoftware/'
PARAMETERS_JSON_PATH = FLIGHT_SOFTWARE_PATH + "utils/parameters.json"
OPNAV_MEDIA_DIR = os.path.join(FLIGHT_SOFTWARE_PATH, "OpticalNavigation/opnav_media/")

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
SYS_TIME = "sys_time"

HARD_SET = "hard_set"

GOM_PIN_STATE = "gom_pin_state"
GOM_PIN_DELAY = "gom_pin_delay"

FILE_PATH = "file_path"
BLOCK_NUMBER = "block_number"
BLOCK_TEXT = "block_text"
TOTAL_BLOCKS = "total_blocks"
REG_ADDRESS = "reg_address"
REG_VALUE = "reg_value"
REG_SIZE = "reg_size"

DET_ENABLE_UINT8 = "det_enable_uint8"
DET0_BIAS_UINT8 = "det0_bias_uint8"
DET1_BIAS_UINT8 = "det1_bias_uint8"
DET0_THRESHOLD_UINT8 = "det0_threshold_uint8"
DET1_THRESHOLD_UINT8 = "det1_threshold_uint8"
RATE_WIDTH_MIN = "rate_width_min"
RATE_WIDTH_MAX = "rate_width_max"
BIN_WIDTH = "bin_width"
BIN_0_MIN_WIDTH = "bin_0_min_width"
RATE_INTERVAL = "rate_interval"
VETO_THRESHOLD_MIN = "veto_threshold_min"
VETO_THRESHOLD_MAX = "veto_threshold_max"
CONFIG_WRITE_PERIOD = "config_write_period"
CONFIG_ROTATE_PERIOD = "config_rotate_period"
DATE_WRITE_PERIOD = "data_write_period"
RATE_DATA_ROTATE_PERIOD = "rate_data_rotate_period"
HISTOGRAM_ROTATE_PERIOD = "histogram_rotate_period"

T_START = "t_start"
T_STOP = "t_stop"

DECIMATION_FACTOR = "decimation_factor"

INDEX = "index"
VBATT = "vbatt"

# Keyword argument definitions for downlink
RTC_TIME = "rtc_time"

ATT_1 = "attitude_1"
ATT_2 = "attitude_2"
ATT_3 = "attitude_3"
ATT_4 = "attitude_4"

HK_TEMP_1 = "hk_temp_1"  # Gomspace temperatures, deg C
HK_TEMP_2 = "hk_temp_2"
HK_TEMP_3 = "hk_temp_3"
HK_TEMP_4 = "hk_temp_4"

GYRO_TEMP = "gyro_temp"  # temperature on the IMU, integer deg C

THERMOCOUPLE_TEMP = "thermo_temp"

CURRENT_IN_1 = "curin_1"  # current coming into the solar converters (mA)
CURRENT_IN_2 = "curin_2"
CURRENT_IN_3 = "curin_3"

VBOOST_1 = "vboost_1"  # voltage of the solar converters (mV)
VBOOST_2 = "vboost_2"
VBOOST_3 = "vboost_3"

SYSTEM_CURRENT = "cursys"  # current that being used by the whole system (mA)
CURSUN = "sun_current"  # current coming into the system (mA)
BATTERY_VOLTAGE = "vbatt"  # battery voltage (mV)
RESERVED1 = "reserved1"  # unknown
CUROUT1 = "curout1"  # current flowing through the controllable output (mA)
CUROUT2 = "curout2"
CUROUT3 = "curout3"
CUROUT4 = "curout4"
CUROUT5 = "curout5"
CUROUT6 = "curout6"
OUTPUTS = "outputs"  # bitmask of the state of the outputs
LATCHUPS1 = "latchup1"  # number of latchup events on each controllable outputs
LATCHUPS2 = "latchup2"
LATCHUPS3 = "latchup3"
LATCHUPS4 = "latchup4"
LATCHUPS5 = "latchup5"
LATCHUPS6 = "latchup6"
WDT_TIME_LEFT_I2C = "wdt_time_i2c"  # seconds (?) left on the I2C watchdog timer
WDT_TIME_LEFT_GND = "wdt_time_gnd"  # seconds (?) left on the dedicated watchdog timer
GOM_BOOTS = "gom_boots"  # number of gomspace reboots
WDT_COUNTS_I2C = "wdt_counts_i2c"  # number of I2C watchdog boots
WDT_COUNTS_GND = "wdt_counts_gnd"  # number of dedicated watchdog boots
GOM_BOOTCAUSE = "bootcause"  # number of gomspace reboots
GOM_BATTMODE = "battmode"  # state machine of the gom. See the manual for more info
GOM_PPT_MODE = (
    "ppt_mode"
)  # power point tracking mode of the solar converters. [1=MPPT, 2=FIXED voltage]
RESERVED2 = "reserved2"  # unknown

RPI_CPU = "rpi_cpu"  # percent utilization of the RPi CPU
RPI_RAM = "rpi_ram"  # percent utilization of the RPi RAM
RPI_DSK = "rpi_disk"  # percent utilization of the RPi's microSD card (Disk)
RPI_TEMP = "rpi_temp"  # temperature on the RPi
RPI_BOOT = "rpi_boot"  # time at which the Pi booted (seconds: unix epoch time)
RPI_UPTIME = "rpi_uptime"  # how many seconds the pi has been up

SUN_CURRENT = "cursun"
BATT_MODE = "batt_mode"

GYROX = "gyro_x"  # gyro rates, degrees/s
GYROY = "gyro_y"
GYROZ = "gyro_z"
MAGX = "mag_x"  # magnetometer readings, microTesla
MAGY = "mag_y"
MAGZ = "mag_z"
ACCX = "acc_x"  # accelerometer readings, m/s^2
ACCY = "acc_y"
ACCZ = "acc_z"

PROP_TANK_PRESSURE = "prs_pressure"  # pressure in the propellant tank

SUCCESSFUL = "successful"

MISSING_BLOCKS = "missing_blocks"
CHECKSUM = "checksum"

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
GYRO_RANGE = 500  # degrees per second

# Gom config command args:
PPT_MODE = "ppt_mode"
BATTHEATERMODE = "battheater_mode"
BATTHEATERLOW = "battheater_low"
BATTHEATERHIGH = "battheater_high"
OUTPUT_NORMAL1 = "output_normal_value1"
OUTPUT_NORMAL2 = "output_normal_value2"
OUTPUT_NORMAL3 = "output_normal_value3"
OUTPUT_NORMAL4 = "output_normal_value4"
OUTPUT_NORMAL5 = "output_normal_value5"
OUTPUT_NORMAL6 = "output_normal_value6"
OUTPUT_NORMAL7 = "output_normal_value7"
OUTPUT_NORMAL8 = "output_normal_value8"
OUTPUT_SAFE1 = "output_safe_value1"
OUTPUT_SAFE2 = "output_safe_value2"
OUTPUT_SAFE3 = "output_safe_value3"
OUTPUT_SAFE4 = "output_safe_value4"
OUTPUT_SAFE5 = "output_safe_value5"
OUTPUT_SAFE6 = "output_safe_value6"
OUTPUT_SAFE7 = "output_safe_value7"
OUTPUT_SAFE8 = "output_safe_value8"
OUTPUT_ON_DELAY = "output_initial_on_delay"
OUTPUT_OFF_DELAY = "output_initial_off_delay"
VBOOST1 = "vboost1"
VBOOST2 = "vboost2"
VBOOST3 = "vboost3"

MAX_VOLTAGE = "max_voltage"
NORM_VOLTAGE = "norm_voltage"
SAFE_VOLTAGE = "safe_voltage"
CRIT_VOLTAGE = "crit_voltage"
OUTPUT_CHANNEL = "output_channel"

CMD = "cmd"
RETURN_CODE = "return_code"

FNAME = "filename"
IGNORE = "ignore"
PASSWORD = "password"

ZERO_WORD = b"\xcb\x51"
ONE_WORD = b"\xdc\x2c"


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
    # SetDesiredAttitude = 2  # arg=attitude # i think this should only be allowed in maneuver mode
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
    #    WhenReorient = 13  # when we want to schedule a reorientation maneuver
    # 2 args, unix time stamp and spin axis vector (2 floats)
    #    ScheduleReorientation = 14
    ScheduleManeuver = 15
    ACSPulsing = 16
    NemoWriteRegister = 17
    NemoReadRegister = 18
    NemoSetConfig = 19
    NemoPowerOff = 20
    NemoPowerOn = 21
    NemoReboot = 22
    NemoProcessRateData = 23
    NemoProcessHistograms = 24
    GomConf1Set = 30
    GomConf1Get = 31
    GomConf2Set = 32
    GomConf2Get = 33

    ShellCommand = 50
    SudoCommand = 51
    Picberry = 52
    ExecPyFile = 53

    IgnoreLowBatt = 60

    # CommandStatus = 99


@unique
class LowBatterySafetyCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    # ExitLBSafetyMode = 1  # no args, # XXX this is an override command
    # SetExitLBSafetyMode = 2  # define battery percentage
    # SetParam = 5
    CritTelem = 6
    BasicTelem = 7
    # DetailedTelem = 8


@unique
class SafetyCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    ExitSafetyMode = 1
    # SetExitSafetyMode = 2
    SetParameter = 5
    CritTelem = 6
    BasicTelem = 7
    DetailedTelem = 8


@unique
class OpNavCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    # RunOpNav = 1  # no args
    # SetInterval = 2  # arg=interval in minutes packed as an int


@unique
class ManeuverCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands


@unique
class SensorsCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    # Thermocouple = 1
    # PressureTransducer = 2
    # Gomspace = 3
    # CameraMux = 4
    # Gyro = 5
    # RTC = 6
    # AX5043 = 7


@unique
class TestCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    # SetTestMode = 1  # no args
    ADCTest = 4
    SeparationTest = 5
    CommsDriver = 7
    RTCTest = 8
    LongString = 9
    PiShutdown = 11


@unique
class CommsCommandEnum(IntEnum):
    Switch = 0  # command for switching flightmode without executing any other commands
    # DownlinkFullDataPacket = 4  # no args
    # SetDataPacket = 5  # arg=data packet id


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
    SetUpdatePath = 9
    AddFileBlock = 10
    GetFileBlocksInfo = 11
    ActivateFile = 12
    ShellCommand = 50
    CeaseComms = 170


@unique
class AttitudeCommandEnum(IntEnum):
    Switch = 0
