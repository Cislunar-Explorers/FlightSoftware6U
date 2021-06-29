# power_structs.py
#
# Desc: defines powerboard structs, conversion
#       functions, and print functions
#
# Author: Daniel Kim (dsk252)
# Debugged by: Tobias Fischer (tmf97) and Stephen Zakoworotny (sjz38) in Spring 2020
#
# Date: 5/31/2020
#
# Status: Completed
#

from ctypes import *
from utils.log import get_log

gom_logger = get_log()


# ----------------------------------------------FORMATTING
# define custom infix operators
class Operator(object):
    def __init__(self, funct):
        self.function = funct

    def __rrshift__(self, other):
        return Operator(lambda x: self.function(other, x))

    def __rshift__(self, other):
        return self.function(other)

    def __ror__(self, other):
        return Operator(lambda x: self.function(other, x))

    def __or__(self, other):
        return self.function(other)

    def __radd__(self, other):
        return Operator(lambda x: self.function(other, x))

    def __add__(self, other):
        return self.function(other)

    def __rmul__(self, other):
        return Operator(lambda x: self.function(other, x))

    def __mul__(self, other):
        return self.function(other)

    def __gt__(self, other):
        return self.function(other)


# pipeline operator (>>_>> or |_|)
_ = Operator(lambda x, y: y(x))


# Color class for formatting
class Color:
    HEADER = "\033[95m"
    BLUE = "\033[94m"
    GREEN = "\033[92m"
    RED = "\033[91m"
    GRAY = "\033[90m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


# ----------------------------------------------STRUCTS
class TestingStruct(BigEndianStructure):
    _fields_ = [("field1", c_uint8), ("field2", c_uint8), ("field3", c_uint16)]


class hkparam_t(BigEndianStructure):
    _fields_ = [
        ("pv", c_uint16 * 3),  # Photo-voltaic input voltage [mV]
        ("pc", c_uint16),  # Total photo current [mA]
        ("bv", c_uint16),  # Battery voltage [mV]
        ("sc", c_uint16),  # Total system current [mA]
        (
            "temp",
            c_int16 * 4,
        ),  # Temp. of boost converters (1,2,3) and onboard battery [degC]
        ("batt_temp", c_int16 * 2),  # External board battery temperatures [degC];
        (
            "latchup",
            c_uint16 * 6,
        ),  # Number of latch-ups on each output 5V and +3V3 channel
        # Order[5V1 5V2 5V3 3.3V1 3.3V2 3.3V3]
        # Transmit as 5V1 first and 3.3V3 last
        ("reset", c_uint8),  # Cause of last EPS reset
        ("bootcount", c_uint16),  # Number of EPS reboots
        ("sw_errors", c_uint16),  # Number of errors in the eps software
        ("ppt_mode", c_uint8),  # 0 = Hardware, 1 = MPPT, 2 = Fixed SW PPT.
        ("channel_status", c_uint8)  # Mask of output channel status, 1=on, 0=off
        # MSB - [QH QS 3.3V3 3.3V2 3.3V1 5V3 5V2 5V1] - LSB
        # QH = Quadbat heater, QS = Quadbat switch
    ]
    # EPS reset cause can be
    #   0. Unknown reset
    #   1. Dedicated WDT reset
    #   2. I2C WDT reset
    #   3. Hard reset
    #   4. Soft reset*
    #   5. Stack overflow
    #   6. Timer overflow
    #   7. Brownout or power-on reset
    #   8. Internal WDT reset


class eps_hk_t(BigEndianStructure):
    _fields_ = [
        ("vboost", c_uint16 * 3),  # Voltage of boost converters [mV] [PV1, PV2, PV3]
        ("vbatt", c_uint16),  # Voltage of battery [mV]
        ("curin", c_uint16 * 3),  # Current in [mA]
        ("cursun", c_uint16),  # Current from boost converters [mA]
        ("cursys", c_uint16),  # Current out of battery [mA]
        ("reserved1", c_uint16),  # Reserved for future use
        ("curout", c_uint16 * 6),  # Current out (switchable outputs) [mA]
        ("output", c_uint8 * 8),  # Status of outputs**
        ("output_on_delta", c_uint16 * 8),  # Time till power on** [s]
        ("output_off_delta", c_uint16 * 8),  # Time till power off** [s]
        ("latchup", c_uint16 * 6),  # Number of latch-ups
        ("wdt_i2c_time_left", c_uint32),  # Time left on I2C wdt [s]
        ("wdt_gnd_time_left", c_uint32),  # Time left on I2C wdt [s]
        ("wdt_csp_pings_left", c_uint8 * 2),  # Pings left on CSP wdt
        ("counter_wdt_i2c", c_uint32),  # Number of WDT I2C reboots
        ("counter_wdt_gnd", c_uint32),  # Number of WDT GND reboots
        ("counter_wdt_csp", c_uint32 * 2),  # Number of WDT CSP reboots
        ("counter_boot", c_uint32),  # Number of EPS reboots
        (
            "temp",
            c_int16 * 6,
        ),  # Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BP4a, BP4b]
        ("bootcause", c_uint8),  # Cause of last EPS reset
        (
            "battmode",
            c_uint8,
        ),  # Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4=full]
        ("pptmode", c_uint8),  # Mode of PPT tracker [1=MPPT, 2=FIXED]
        ("reserved2", c_uint16),
    ]


class eps_hk_vi_t(BigEndianStructure):
    _fields_ = [
        ("vboost", c_uint16 * 3),  # Voltage of boost converters [mV] [PV1, PV2, PV3]
        ("vbatt", c_uint16),  # Voltage of battery [mV]
        ("curin", c_uint16 * 3),  # Current in [mA]
        ("cursun", c_uint16),  # Current from boost converters [mA]
        ("cursys", c_uint16),  # Current out of battery [mA]
        ("reserved1", c_uint16),  # Reserved for future use
    ]


class eps_hk_out_t(BigEndianStructure):
    _fields_ = [
        ("curout", c_uint16 * 6),  # Current out (switchable outputs) [mA]
        ("output", c_uint8 * 8),  # Status of outputs**
        ("output_on_delta", c_uint16 * 8),  # Time till power on** [s]
        ("output_off_delta", c_uint16 * 8),  # Time till power off** [s]
        ("latchup", c_uint16 * 6),  # Number of latch-ups
    ]


class eps_hk_wdt_t(BigEndianStructure):
    _fields_ = [
        ("wdt_i2c_time_left", c_uint32),  # Time left on I2C wdt [s]
        ("wdt_gnd_time_left", c_uint32),  # Time left on I2C wdt [s]
        ("wdt_csp_pings_left", c_uint8 * 2),  # Pings left on CSP wdt
        ("counter_wdt_i2c", c_uint32),  # Number of WDT I2C reboots
        ("counter_wdt_gnd", c_uint32),  # Number of WDT GND reboots
        ("counter_wdt_csp", c_uint32 * 2),  # Number of WDT CSP reboots
    ]


class eps_hk_basic_t(BigEndianStructure):
    _fields_ = [
        ("counter_boot", c_uint32),  # Number of EPS reboots
        (
            "temp",
            c_int16 * 6,
        ),  # Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BATT0, BATT1]
        ("bootcause", c_uint8),  # Cause of last EPS reset
        (
            "battmode",
            c_uint8,
        ),  # Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4=full]
        ("pptmode", c_uint8),  # Mode of PPT tracker [1=MPPT, 2=FIXED]
        ("reserved2", c_uint16),
    ]


class eps_config_t(BigEndianStructure):
    _fields_ = [
        ("ppt_mode", c_uint8),  # Mode for PPT [1 = AUTO, 2 = FIXED]
        ("battheater_mode", c_uint8),  # Mode for battheater [0 = Manual, 1 = Auto]
        ("battheater_low", c_int8),  # Turn heater on at [degC]
        ("battheater_high", c_int8),  # Turn heater off at [degC]
        ("output_normal_value", c_uint8 * 8),  # Nominal mode output value
        ("output_safe_value", c_uint8 * 8),  # Safe mode output value
        (
            "output_initial_on_delay",
            c_uint16 * 8,
        ),  # Output switches: init with these on delays [s]
        (
            "output_initial_off_delay",
            c_uint16 * 8,
        ),  # Output switches: init with these on delays [s]
        ("vboost", c_uint16 * 3),  # Fixed PPT point for boost converters [mV]
    ]


class eps_config2_t(BigEndianStructure):
    _fields_ = [
        ("batt_maxvoltage", c_uint16),
        ("batt_safevoltage", c_uint16),
        ("batt_criticalvoltage", c_uint16),
        ("batt_normalvoltage", c_uint16),
        ("reserved1", c_uint32 * 2),
        ("reserved2", c_uint8 * 4),
    ]


# ----------------------------------------------HELPERS
# returns true if s is of type struct
def isStruct(s):
    allStructs = [
        TestingStruct,
        hkparam_t,
        eps_hk_t,
        eps_hk_vi_t,
        eps_hk_out_t,
        eps_hk_wdt_t,
        eps_hk_basic_t,
        eps_config_t,
        eps_config2_t,
    ]
    return type(s) in allStructs


# returns true if arr is a ctypes byte array with the
# specified number of elements.
def isCByteArray(arr):
    return type(arr).__name__[:13] == "c_byte_Array_"


# takes a string [s] and returns the appropriate struct;
# returns TestingStruct if string does not match any struct
# raises: AssertionError if s is not a string
def structMaker(s):
    assert type(s) is str

    struct_options = {
        "hkparam_t": hkparam_t(),
        "eps_hk_t": eps_hk_t(),
        "eps_hk_vi_t": eps_hk_vi_t(),
        "eps_hk_out_t": eps_hk_out_t(),
        "eps_hk_wdt_t": eps_hk_wdt_t(),
        "eps_hk_basic_t": eps_hk_basic_t(),
        "eps_config_t": eps_config_t(),
        "eps_config2_t": eps_config2_t(),
    }
    try:
        return struct_options[s]
    except KeyError:
        gom_logger.warning(f"In structMaker: input '{s}' not valid. Return TestStruct")
        return TestingStruct()


# struct -> c_bytearray
# takes a struct [s] and converts it into a ctypes
# bytearray of the appropriate size.
# note: bytes are stored in order, e.g. the first field
# of the struct is stored starting in index 0 of the
# byte array and so on.
# raises: AssertionError if s is not a struct
def c_structToByteArray(s):
    assert isStruct(s)
    byteArray = (c_byte * (sizeof(s)))()
    spoint = pointer(s)
    cpoint = pointer(byteArray)
    memmove(cpoint, spoint, sizeof(s))
    return byteArray


# c_bytearray -> bytearray
# takes a ctypes bytearray [b] and converts it into a
# python bytearray of the appropriate size, adjusting
# for negative values (although it shouldn't matter in the end)
# raises: AssertionError if b is not a ctypes byte array
def c_byteArrayToBytes(b):
    # assert isCByteArray(b)
    acc = []
    for n in b:
        if n < 0:
            acc += [256 + n]  # adjust for negative values
        else:
            acc += [n]
    return bytearray(acc)


# c_bytearray -> struct
# takes a ctypes bytearray [b] and converts it into a
# struct [s]. Raises assertion error if the size of the
# bytearray does not match the size of the struct.
# raises: AssertionError if s is not a string
#         AssertionError if b is not correct size
def c_byteArrayToStruct(b, s):
    assert sizeof(b) == sizeof(structMaker(s))
    bpoint = pointer(b)
    struct = structMaker(s)
    spoint = pointer(struct)
    memmove(spoint, bpoint, sizeof(b))
    return struct


# takes an int [i] and outputs a python bytearray with the
# int divided into [num] number of bytes
# raises: AssertionError if i or num are not ints
def toBytes(i, num):
    assert type(i) is int and type(num) is int
    binary = bin(i)[2:]
    rem = len(binary) % 8
    binary = "0" * (8 - rem) + binary  # add zeros to make it 8 bit
    size = int(len(binary) / 8)  # Cast to int added during testing
    bytes = (
        "0" * 8 * (num - size) + binary
    )  # add zeros to make it into right num of bytes

    acc = []
    for n in range(num):
        acc += [int(bytes[8 * n: 8 * (n + 1)], 2)]

    return bytearray(acc)


# bytearray -> c_bytearray takes a ctypes bytearray [i] and converts it into a python
# bytearray. raises: AssertionError if i is not a ctypes bytearray
def c_bytesToByteArray(i):
    assert type(i).__name__ == "bytearray"
    return (c_byte * len(i))(*i)


# struct -> c_bytearray -> bytearray
# converts a struct [s] into a python bytearray of the appropriate size.
# raises: AssertionError if s is not a struct
def c_structToBytes(s):
    assert isStruct(s)
    return s >> _ >> c_structToByteArray >> _ >> c_byteArrayToBytes


# bytearray -> c_bytearray -> struct
# converts a python bytearray [i] to a struct [s].
# raises: TODO: how to check types of ctypes
def c_bytesToStruct(i: bytearray, s: str):
    assert type(i) == bytearray
    assert type(s) == str
    return c_byteArrayToStruct(c_bytesToByteArray(i), s)


# bytearray -> byte[]
# converts a python bytearray [b] to a python list.
# raises: AssertionError if b is not a python bytearray
def bytesToList(b):
    assert type(b) == bytearray
    acc = []
    for n in b:
        acc += [n]
    return acc


# ----------------------------------------------DISPLAY
# color functions
# were rewritten from lambda functions on 2020-08-24. Need to be tested!


def B(x) -> str:
    return Color.BOLD + str(x) + Color.ENDC


def BL(x) -> str:
    return Color.BLUE + str(x) + Color.ENDC


def W(x) -> str:
    return Color.WARNING + str(x) + Color.ENDC


def G(x) -> str:
    return Color.GREEN + str(x) + Color.ENDC


def F(x) -> str:
    return Color.FAIL + str(x) + Color.ENDC


def R(x) -> str:
    return Color.RED + str(x) + Color.ENDC


def GR(x) -> str:
    return Color.GRAY + str(x) + Color.ENDC


def mv(x: str) -> str:
    return str(x) + "mV"


def ma(x: str) -> str:
    return str(x) + "mA"


def degc(x: str) -> str:
    return str(x) + "degC"


def RES(x):
    reset_reasons = {
        0: "Unknown reset",
        1: "Dedicated WDT reset",
        2: "I2C WDT reset",
        3: "Hard reset",
        4: "Soft reset",
        5: "Stack overflow reset",
        6: "Timer overflow reset",
        7: "Brownout or power-on reset",
        8: "Internal WDT reset",
    }
    return reset_reasons.get(x, "ERROR")


# prints housekeeping info given hkparam_t struct
def displayHK(hk):
    assert type(hk) == hkparam_t

    def mult(x, y):
        return x * y

    def add(x, y):
        return x + y

    def substr(x, y):
        return y[x:]

    gom_logger.info(G("***************-HOUSEKEEPING-***************"))
    gom_logger.info(
        GR("Photo-voltaic inputs:        ")
        + "1-%s 2-%s 3-%s" % (R(mv(hk.pv[0])), R(mv(hk.pv[1])), R(mv(hk.pv[2])))
    )

    gom_logger.info(GR("Total photo current:         ") + "%s" % (R(ma(hk.pc))))
    gom_logger.info(GR("Battery voltage:             ") + "%s" % (R(mv(hk.bv))))
    gom_logger.info(GR("Total system current:        ") + "%s" % (R(ma(hk.sc))))

    gom_logger.info(
        GR("Temp of boost converters:    ")
        + "1-%s 2-%s 3-%s batt-%s"
        % (
            R(degc(hk.temp[0])),
            R(degc(hk.temp[1])),
            R(degc(hk.temp[2])),
            R(degc(hk.temp[3])),
        )
    )

    gom_logger.info(
        GR("External batt temp:          ")
        + "1-%s 2-%s" % (R(degc(hk.batt_temp[0])), R(degc(hk.batt_temp[1])))
    )

    gom_logger.info(
        GR("Latchups:                    ")
        + "1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s]"
        % (
            R(hk.latchup[0]),
            R(hk.latchup[1]),
            R(hk.latchup[2]),
            R(hk.latchup[3]),
            R(hk.latchup[4]),
            R(hk.latchup[5]),
        )
    )

    gom_logger.info(GR("Cause of last reset:         ") + "%s" % (R(RES(hk.reset))))
    gom_logger.info(GR("Number of reboots:           ") + "%s" % (R(hk.bootcount)))
    gom_logger.info(GR("Number of software errors:   ") + "%s" % (R(hk.sw_errors)))
    gom_logger.info(GR("PPT mode:                    ") + "%s" % (R(hk.ppt_mode)))
    # gom_logger.info(
    #    GR("Channel output:              ")
    #    + "%s"
    #    % R(
    #        (
    #                hk.channel_status # I will give $5 to whoever can explain wtf is going on here
    #                >> _
    #                >> bin
    #                >> _
    #                >> substr(2)
    #                >> _
    #                >> len
    #                >> _
    #                >> mult(-1)
    #                >> _
    #                >> add(8)
    #                >> _
    #                >> mult("0")
    #        )
    #        + (hk.channel_status >> _ >> bin >> _ >> str >> _ >> substr(2))
    #    )
    #)


# prints config info given eps_config_t struct
def displayConfig(conf):
    assert type(conf) == eps_config_t

    def pptmode(x: int):
        gom_logger.debug("PPT Mode code: %s", x)
        return {1: "AUTO[1]", 2: "FIXED[2]"}.get(x, "ERROR")

    def battheatermode(x: int):
        gom_logger.debug("BattHeater Mode code: %s", x)
        return {0: "MANUAL[0]", 1: "AUTO[1]"}.get(x, "ERROR")

    def sec(x: str):
        return str(x) + "s"

    gom_logger.info(G("***************-CONFIG-***************"))
    gom_logger.info(
        GR("PPT mode:                    ") + "%s" % (R(pptmode(conf.ppt_mode)))
    )
    gom_logger.info(
        GR("Battheater mode:             ")
        + "%s" % (R(battheatermode(conf.battheater_mode)))
    )
    gom_logger.info(
        GR("Battheater low:              ") + "%s" % (R(degc(conf.battheater_low)))
    )
    gom_logger.info(
        GR("Battheater high:             ") + "%s" % (R(degc(conf.battheater_high)))
    )

    gom_logger.info(
        GR("Nominal mode output value:   ")
        + "1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]"
        % (
            R(conf.output_normal_value[0]),
            R(conf.output_normal_value[1]),
            R(conf.output_normal_value[2]),
            R(conf.output_normal_value[3]),
            R(conf.output_normal_value[4]),
            R(conf.output_normal_value[5]),
            R(conf.output_normal_value[6]),
            R(conf.output_normal_value[7]),
        )
    )

    gom_logger.info(
        GR("Safe mode output value:      ")
        + "1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]"
        % (
            R(conf.output_safe_value[0]),
            R(conf.output_safe_value[1]),
            R(conf.output_safe_value[2]),
            R(conf.output_safe_value[3]),
            R(conf.output_safe_value[4]),
            R(conf.output_safe_value[5]),
            R(conf.output_safe_value[6]),
            R(conf.output_safe_value[7]),
        )
    )

    gom_logger.info(
        GR("Output initial on:           ")
        + "1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]"
        % (
            R(sec(conf.output_initial_on_delay[0])),
            R(sec(conf.output_initial_on_delay[1])),
            R(sec(conf.output_initial_on_delay[2])),
            R(sec(conf.output_initial_on_delay[3])),
            R(sec(conf.output_initial_on_delay[4])),
            R(sec(conf.output_initial_on_delay[5])),
            R(sec(conf.output_initial_on_delay[6])),
            R(sec(conf.output_initial_on_delay[7])),
        )
    )

    gom_logger.info(
        GR("Output initial off:          ")
        + "1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]"
        % (
            R(sec(conf.output_initial_off_delay[0])),
            R(sec(conf.output_initial_off_delay[1])),
            R(sec(conf.output_initial_off_delay[2])),
            R(sec(conf.output_initial_off_delay[3])),
            R(sec(conf.output_initial_off_delay[4])),
            R(sec(conf.output_initial_off_delay[5])),
            R(sec(conf.output_initial_off_delay[6])),
            R(sec(conf.output_initial_off_delay[7])),
        )
    )

    gom_logger.info(
        GR("PPT point for boost conv:    ")
        + "1-%s 2-%s 3-%s"
        % (R(mv(conf.vboost[0])), R(mv(conf.vboost[1])), R(mv(conf.vboost[2])))
    )


# prints config2 info given eps_config2_t struct
def displayConfig2(conf):
    assert type(conf) == eps_config2_t
    gom_logger.info(G("***************-CONFIG2-***************"))
    gom_logger.info(
        GR("Batt Max Voltage:            ") + "%s" % (R(mv(conf.batt_maxvoltage)))
    )
    gom_logger.info(
        GR("Batt Safe Voltage:           ") + "%s" % (R(mv(conf.batt_safevoltage)))
    )
    gom_logger.info(
        GR("Batt Critical Voltage:       ") + "%s" % (R(mv(conf.batt_criticalvoltage)))
    )
    gom_logger.info(
        GR("Batt Normal Voltage:         ") + "%s" % (R(mv(conf.batt_normalvoltage)))
    )


def displayHk2(hk2):
    assert type(hk2) is eps_hk_t

    def RJ4(x):
        return str(x).rjust(4, " ")

    def RJ5(x):
        return str(x).rjust(5, " ")

    def batt_mode():
        modes = {
            0: BL("Initial"),
            1: F("Undervoltage"),
            2: W("Safe Mode"),
            3: G("Nominal"),
            4: G("Full"),
        }
        return B(modes.get(hk2.battmode, "ERROR").ljust(13 + 9))

    def channel_state(i):
        return f"--> EN:{hk2.output[i]} [{RJ4(hk2.curout[i])}, {RJ4(hk2.latchup[i])},{RJ5(hk2.output_on_delta[i])},{RJ5(hk2.output_off_delta[i])}]"

    gom_logger.info(
        "\nInputs	               |=======|         Outputs            I(mA), LUPs, ttON, ttOFF\n"
        " 1:              +-------------------+   0 (H1-47) %s\n" % channel_state(0)
        + "   %s mV ->    |  Voltage          |\n" % RJ4(hk2.vboost[0])
        + "   %s mA ->    |  %s mV         |   1 (H1-49) %s\n"
        % (RJ4(hk2.curin[0]), B(RJ5(hk2.vbatt)), channel_state(1))
        + "   %s mW ->    |                   |\n"
        % RJ4(hk2.curin[0] * hk2.vboost[0] // 1000)
        + "                 |  Input            |   2 (H1-51) %s\n" % channel_state(2)
        + " 2:              |  %s mA         |\n" % RJ5(hk2.cursun)
        + "   %s mV ->    |  %s mW         |   3 (H1-48) %s\n"
        % (RJ4(hk2.vboost[1]), RJ5(hk2.vbatt * hk2.cursun // 1000), channel_state(3))
        + "   %s mA ->    |                   |\n" % RJ4(hk2.curin[1])
        + "   %s mW ->    |  Output           |   4 (H1-50) %s\n"
        % (RJ4(hk2.curin[1] * hk2.vboost[1] // 1000), channel_state(4))
        + "                 |  %s mA         |\n" % RJ5(hk2.cursys)
        + " 3:              |  %s mW	     |   5 (H1-52) %s\n"
        % (RJ5(hk2.cursys * hk2.vbatt // 1000), channel_state(5))
        + "   %s mV ->    |                   |\n" % RJ4(hk2.vboost[2])
        + "   %s mA ->    |  Mode             |   6         --> EN:%d\n"
        % (RJ4(hk2.curin[2]), hk2.output[6])
        + "   %s mW ->    |  %s: %s |\n"
        % (RJ4(hk2.curin[2] * hk2.vboost[2] // 1000), str(hk2.battmode), batt_mode())
        + "                 +-------------------+   7         --> EN:%d\n"
        % hk2.output[7]
    )
    gom_logger.info(
        f"\n"
        f"                 1    2    3    4    5    6\n"
        f"Temp (degC): {''.join([RJ5(t) for t in hk2.temp])}\n"
        f"\n"
        f"Boots\n"
        f"Counts: {RJ5(hk2.counter_boot)}\n"
        f"Last Cause: {hk2.bootcause} ({RES(hk2.bootcause)})\n"
        f"\n"
        f"WDTs      i2c   gnd  csp1  csp2\n"
        f"Count:  {RJ5(hk2.counter_wdt_i2c)} {RJ5(hk2.counter_wdt_gnd)} {RJ5(hk2.counter_wdt_csp[0])} {RJ5(hk2.counter_wdt_csp[1])}\n"
        f" Left:  {RJ5(hk2.wdt_i2c_time_left)} {RJ5(hk2.wdt_gnd_time_left)} {RJ5(hk2.wdt_csp_pings_left[0])} {RJ5(hk2.wdt_csp_pings_left[1])}\n"
    )


def displayStruct(s):
    assert isStruct(s)
    for i in s._fields_:
        b = getattr(s, i[0])
        try:
            gom_logger.info(f"{i[0]}: {b[:]}")
        except TypeError:
            gom_logger.info(f"{i[0]}: {b}")


# ----------------------------------------------CONSTANTS
CONFIG_DEFAULT = eps_config_t()
CONFIG_DEFAULT.ppt_mode = 2
CONFIG_DEFAULT.battheater_high = 5
CONFIG_DEFAULT.vboost[0] = 3700
CONFIG_DEFAULT.vboost[1] = 3700
CONFIG_DEFAULT.vboost[2] = 3700
