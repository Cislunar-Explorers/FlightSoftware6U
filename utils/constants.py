import os
from enum import Enum, IntEnum, unique
from pathlib import Path
from typing import cast

from dotenv import dotenv_values

# dotenv vars
config = dotenv_values()

# absolute project root path
FLIGHT_SOFTWARE_PATH = (Path(__file__).parent / "..").resolve()

CISLUNAR_BASE_DIR = cast("str", config.get("CISLUNAR_BASE_DIR", "cislunar_data"))
if not os.path.isabs(CISLUNAR_BASE_DIR):
    CISLUNAR_BASE_DIR = os.path.join(FLIGHT_SOFTWARE_PATH, CISLUNAR_BASE_DIR)
os.makedirs(CISLUNAR_BASE_DIR, exist_ok=True)

SURRENDER_LOCAL_DIR = cast("str", config.get("SURRENDER_LOCAL_DIR"))

FOR_FLIGHT = config["FOR_FLIGHT"] == "1"
LOG = config["LOG"] == "1"
TEST = config.get("TEST", "0") == "1"

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

# Whether or not we actually do bit inflation:
INFLATION: bool = False

# Serialization Sizes
ID_SIZE = 1
COUNTER_SIZE = 3
DATA_LEN_SIZE = 1
MIN_COMMAND_SIZE = MAC_LENGTH + COUNTER_SIZE + ID_SIZE + DATA_LEN_SIZE

MAX_COMMAND_SIZE = 200  # Maximum command size based on the radio board's buffer size.
# TODO: figure out what this number _actually_ is

# Serializations Offsets
MAC_OFFSET = 0
COUNTER_OFFSET = MAC_OFFSET + MAC_LENGTH
ID_OFFSET = COUNTER_SIZE + COUNTER_OFFSET
DATA_LEN_OFFSET = ID_SIZE + ID_OFFSET
DATA_OFFSET = DATA_LEN_SIZE + DATA_LEN_OFFSET

# Important paths
# FLIGHT_SOFTWARE_PATH = '/home/pi/FlightSoftware/'
PARAMETERS_JSON_PATH = os.path.join(FLIGHT_SOFTWARE_PATH, "utils/parameters.json")
OPNAV_MEDIA_DIR = os.path.join(FLIGHT_SOFTWARE_PATH, "opnav/opnav_media/")


class StringEnum(str, Enum):
    """Similar to the built-in IntEnum, but with strings!
    See https://docs.python.org/3/library/enum.html#others for inspiration."""

    pass


# Keyword Argument Definitions for Commands
@unique
class CommandKwargs(StringEnum):
    """Enum of all the keyword argument names that are used in commands"""

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

    # TIME = "time"
    SYS_TIME = "sys_time"
    MANEUVER_TIME = "maneuver_time"

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

    CMD = "cmd"
    FNAME = "filename"
    IGNORE = "ignore"
    PASSWORD = "password"

    EARTH_THRESH = "earth_threshold"
    MOON_THRESH = "moon_threshold"
    SUN_THRESH = "sun_threshold"

    OUTPUT_CHANNEL = "output_channel"


# Keyword argument definitions for downlinks
@unique
class DownlinkKwargs(StringEnum):
    """Enum of all the keyword argument names used for downlinking."""

    VALUE = "value"

    RTC_TIME = "rtc_time"
    TIME = "time"

    # Components of the attitude quaternion result from opnav
    ATT_1 = "attitude_1"
    ATT_2 = "attitude_2"
    ATT_3 = "attitude_3"
    ATT_4 = "attitude_4"

    # Components of the position results from opnav
    POS_X = "position_x"
    POS_Y = "position_y"
    POS_Z = "position_z"

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
    # seconds (?) left on the I2C watchdog timer
    WDT_TIME_LEFT_I2C = "wdt_time_i2c"
    # seconds (?) left on the dedicated watchdog timer
    WDT_TIME_LEFT_GND = "wdt_time_gnd"
    GOM_BOOTS = "gom_boots"  # number of gomspace reboots
    WDT_COUNTS_I2C = "wdt_counts_i2c"  # number of I2C watchdog boots
    WDT_COUNTS_GND = "wdt_counts_gnd"  # number of dedicated watchdog boots
    GOM_BOOTCAUSE = "bootcause"  # number of gomspace reboots
    GOM_BATTMODE = "battmode"  # state machine of the gom. See the manual for more info
    # power point tracking mode of the solar converters. [1=MPPT, 2=FIXED voltage]
    GOM_PPT_MODE = "ppt_mode"
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
    RETURN_CODE = "return_code"


# Constants related to the CQC competition
a = 1664525
b = 1013904223
M = 2 ** 32
team_identifier = 0xEB902D2D  # Team 2

# TODO: validate these values:
SPLIT_BURNWIRE_DURATION = 1.5  # second
ANTENNAE_BURNWIRE_DURATION = 1  # second
BURN_WAIT_TIME = 1  # minutes changed to glow wait time in params

GOM_TIMING_FUDGE_FACTOR = 3  # milliseconds

MAX_GYRO_RATE = 250  # degrees/sec # TODO

NO_FM_CHANGE = -1

# Gyro specific constants
GYRO_RANGE = 500  # degrees per second

# Gom config command args:


@unique
class GomConfKwargs(StringEnum):
    """Enum of all the arguments in the P31u's config"""

    # Conf1:
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

    # Conf2:
    MAX_VOLTAGE = "max_voltage"
    NORM_VOLTAGE = "norm_voltage"
    SAFE_VOLTAGE = "safe_voltage"
    CRIT_VOLTAGE = "crit_voltage"


# Bit inflation definition
# TODO: move to parameters
ZERO_WORD = b"\xcb\x51"
ONE_WORD = b"\xdc\x2c"


# GOMspace Channel designations:
# TODO: re-evaluate and double check before flight for each satellite half
@unique
class GomOutputs(IntEnum):
    """IntEnum class defining the outputs of the GOMSpace P31u"""

    comms = 0
    burnwire_1 = 1
    glowplug_2 = 2
    # I know this looks backwards, but this is correct for historical reasons
    glowplug_1 = 3
    solenoid = 4
    electrolyzer = 5


@unique
class FMEnum(IntEnum):
    """Enumerations of the flight modes (states) of FSW"""

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
class CommandEnum(IntEnum):
    """Enum of every command that we can execute"""

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
    CommandMode = 10
    AttitudeAdjustment = 11

    SetElectrolysis = 12  # arg = bool whether to start or stop electrolysis
    SetParam = 13
    CritTelem = 14
    BasicTelem = 15
    DetailedTelem = 16
    # CqcVerification = 17
    GetParam = 18
    SetOpnavInterval = 19
    ScheduleManeuver = 20
    ACSPulsing = 21
    NemoWriteRegister = 22
    NemoReadRegister = 23
    NemoSetConfig = 24
    NemoPowerOff = 25
    NemoPowerOn = 26
    NemoReboot = 27
    NemoProcessRateData = 28
    NemoProcessHistograms = 29
    GomConf1Set = 30
    GomConf1Get = 31
    GomConf2Set = 32
    GomConf2Get = 33

    SetUpdatePath = 34
    AddFileBlock = 35
    GetFileBlocksInfo = 36
    ActivateFile = 37

    SetSystemTime = 38
    RebootPi = 39

    RebootGom = 40
    PowerCycle = 41
    GomPin = 42
    # GomGeneralCmd = 43
    # GeneralCmd = 44

    LowBattThresh = 45
    ScheduleOpnav = 46

    ShellCommand = 50
    SudoCommand = 51
    Picberry = 52
    ExecPyFile = 53
    PiShutdown = 54

    SeparationTest = 55
    LongString = 56

    IgnoreLowBatt = 60
    SetEMSThresh = 61
    CeaseComms = 170
