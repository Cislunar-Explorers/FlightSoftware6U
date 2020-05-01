# power_structs.py
#
# Desc: defines powerboard structs, conversion
#       functions, and print functions
#
# Author: Daniel Kim (dsk252)
# 
# Date: 10/3/16
#
# Status: Completed
#

from ctypes import *


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
_ = Operator(lambda x, y: y (x))

# Color class for formatting
class Color:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    RED = '\033[91m'
    GRAY = '\033[90m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

# ----------------------------------------------STRUCTS
class TestingStruct(BigEndianStructure):
    _fields_ = [
        ("field1", c_uint8),
        ("field2", c_uint8),
        ("field3", c_uint16)
    ]

class hkparam_t(BigEndianStructure):
    _fields_ = [
        ("pv",              c_uint16*3),    # Photo-voltaic input voltage [mV]
        ("pc",              c_uint16),      # Total photo current [mA]
        ("bv",              c_uint16),      # Battery voltage [mV]
        ("sc",              c_uint16),      # Total system current [mA]
        ("temp",            c_int16*4),     # Temp. of boost converters (1,2,3) and onboard battery [degC]
        ("batt_temp",       c_int16*2),     # External board battery temperatures [degC];
        ("latchup",         c_uint16*6),    # Number of latch-ups on each output 5V and +3V3 channel
                                            # Order[5V1 5V2 5V3 3.3V1 3.3V2 3.3V3]
                                            # Transmit as 5V1 first and 3.3V3 last
        ("reset",           c_uint8),       # Cause of last EPS reset
        ("bootcount",       c_uint16),      # Number of EPS reboots
        ("sw_errors",       c_uint16),      # Number of errors in the eps software
        ("ppt_mode",        c_uint8),       # 0 = Hardware, 1 = MPPT, 2 = Fixed SW PPT.
        ("channel_status",  c_uint8)        # Mask of output channel status, 1=on, 0=off
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
        ("vboost",              c_uint16*3),    # Voltage of boost converters [mV] [PV1, PV2, PV3]
        ("vbatt",               c_uint16),      # Voltage of battery [mV]
        ("curin",               c_uint16*3),    # Current in [mA]
        ("cursun",              c_uint16),      # Current from boost converters [mA]
        ("cursys",              c_uint16),      # Current out of battery [mA]
        ("reserved1",           c_uint16),      # Reserved for future use
        ("curout",              c_uint16*6),    # Current out (switchable outputs) [mA] 
        ("output",              c_uint8*8),     # Status of outputs**
        ("output_on_delta",     c_uint16*8),    # Time till power on** [s] 
        ("output_off_delta",    c_uint16*8),    # Time till power off** [s]
        ("latchup",             c_uint16*6),    # Number of latch-ups
        ("wdt_i2c_time_left",   c_uint32),      # Time left on I2C wdt [s] 
        ("wdt_gnd_time_left",   c_uint32),      # Time left on I2C wdt [s] 
        ("wdt_csp_pings_left",  c_uint8*2),     # Pings left on CSP wdt
        ("counter_wdt_i2c",     c_uint32),      # Number of WDT I2C reboots 
        ("counter_wdt_gnd",     c_uint32),      # Number of WDT GND reboots 
        ("counter_wdt_csp",     c_uint32*2),    # Number of WDT CSP reboots 
        ("counter_boot",        c_uint32),      # Number of EPS reboots
        ("temp",                c_int16*6),     # Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BP4a, BP4b]
        ("bootcause",           c_uint8),       # Cause of last EPS reset
        ("battmode",            c_uint8),       # Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4=full]
        ("pptmode",             c_uint8),       # Mode of PPT tracker [1=MPPT, 2=FIXED]
        ("reserved2",           c_uint16)
    ]
 
class eps_hk_vi_t(BigEndianStructure):
    _fields_ = [
        ("vboost",      c_uint16*3),        # Voltage of boost converters [mV] [PV1, PV2, PV3]
        ("vbatt",       c_uint16),          # Voltage of battery [mV]
        ("curin",       c_uint16*3),        # Current in [mA]
        ("cursun",      c_uint16),          # Current from boost converters [mA] 
        ("cursys",      c_uint16),          # Current out of battery [mA]
        ("reserved1",   c_uint16)           # Reserved for future use
    ]
    
class eps_hk_out_t(BigEndianStructure):
    _fields_ = [
        ("curout",          c_uint16*6),    # Current out (switchable outputs) [mA] 
        ("output",          c_uint8*8),     # Status of outputs**
        ("output_on_delta", c_uint16*8),    # Time till power on** [s]
        ("output_off_delta",c_uint16*8),    # Time till power off** [s]
        ("latchup",         c_uint16*6),    # Number of latch-ups
    ]

class eps_hk_wdt_t(BigEndianStructure):
    _fields_ = [
        ("wdt_i2c_time_left",   c_uint32),  # Time left on I2C wdt [s] 
        ("wdt_gnd_time_left",   c_uint32),  # Time left on I2C wdt [s] 
        ("wdt_csp_pings_left",  c_uint8*2), # Pings left on CSP wdt
        ("counter_wdt_i2c",     c_uint32),  # Number of WDT I2C reboots 
        ("counter_wdt_gnd",     c_uint32),  # Number of WDT GND reboots 
        ("counter_wdt_csp",     c_uint32*2) # Number of WDT CSP reboots
    ]

class eps_hk_basic_t(BigEndianStructure):
    _fields_ = [
        ("counter_boot",    c_uint32),      # Number of EPS reboots
        ("temp",            c_int16*6),     # Temperatures [degC] [0 = TEMP1, TEMP2, TEMP3, TEMP4, BATT0, BATT1]
        ("bootcause",       c_uint8),       # Cause of last EPS reset
        ("battmode",        c_uint8),       # Mode for battery [0 = initial, 1 = undervoltage, 2 = safemode, 3 = nominal, 4=full]
        ("pptmode",         c_uint8),       # Mode of PPT tracker [1=MPPT, 2=FIXED]
        ("reserved2",       c_uint16)
    ]

class eps_config_t(BigEndianStructure):
    _fields_ = [
        ("ppt_mode",                    c_uint8),       # Mode for PPT [1 = AUTO, 2 = FIXED]
        ("battheater_mode",             c_uint8),       # Mode for battheater [0 = Manual, 1 = Auto]
        ("battheater_low",              c_int8),        # Turn heater on at [degC]
        ("battheater_high",             c_int8),        # Turn heater off at [degC]
        ("output_normal_value",         c_uint8*8),     # Nominal mode output value
        ("output_safe_value",           c_uint8*8),     # Safe mode output value
        ("output_initial_on_delay",     c_uint16*8),    # Output switches: init with these on delays [s]
        ("output_initial_off_delay",    c_uint16*8),    # Output switches: init with these on delays [s]
        ("vboost",                      c_uint16*3)     # Fixed PPT point for boost converters [mV]
    ]

class eps_config2_t(BigEndianStructure):
    _fields_ = [
        ("batt_maxvoltage",         c_uint16),
        ("batt_safevoltage",        c_uint16),
        ("batt_criticalvoltage",    c_uint16),
        ("batt_normalvoltage",      c_uint16),
        ("reserved1",               c_uint32*2),
        ("reserved2",               c_uint8*4)
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
        eps_config2_t
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
    assert type(s) == str
    if s == "hkparam_t": return hkparam_t()
    if s == "eps_hk_t": return eps_hk_t()
    if s == "eps_hk_vi_t": return eps_hk_vi_t()
    if s == "eps_hk_out_t": return eps_hk_out_t()
    if s == "eps_hk_wdt_t": return eps_hk_wdt_t()
    if s == "eps_hk_basic_t": return eps_hk_basic_t()
    if s == "eps_config_t": return eps_config_t()
    if s == "eps_config2_t": return eps_config2_t()
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
    byteArray = (c_byte*(sizeof(s))) ()
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
    #assert isCByteArray(b)
    acc = []
    for n in b:
        if n < 0: acc += [256+n]    # adjust for negative values
        else: acc += [n]
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
    assert type(i) == int and type(num) == int
    binary = bin(i)[2:]
    rem = len(binary) % 8 
    binary = '0'*(8-rem)+binary     # add zeros to make it 8 bit
    size = int(len(binary)/8)	    # Cast to int added during testing
    bytes = '0'*8*(num-size)+binary # add zeros to make it into right num of bytes

    acc = []
    for n in range(num):
        acc += [int(bytes[8*n:8*(n+1)], 2)]

    return bytearray(acc)


# TODO: This function is in very very bad shape. Need to change. It's essentially the same as c_byteArrayToBytes but
#  with contradicting documentation. The assert statements causes powertest.py to fail, but runs fine with the assert
#  statements commented out

# bytearray -> c_bytearray takes a ctypes bytearray [i] and converts it into a python
# bytearray. raises: AssertionError if i is not a ctypes bytearray
def c_bytesToByteArray(i):
    #assert isCByteArray(i)
    assert type(i).__name__ == "bytearray"
    return (c_byte*len(i))(*i)


# struct -> c_bytearray -> bytearray
# converts a struct [s] into a python bytearray of the appropriate size.
# raises: AssertionError if s is not a struct
def c_structToBytes(s):
    assert isStruct(s)
    return s >>_>> c_structToByteArray >>_>> c_byteArrayToBytes

# bytearray -> c_bytearray -> struct
# converts a python bytearray [i] to a struct [s].
# raises: TODO: how to check types of ctypes
def c_bytesToStruct(i, s):
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
B = lambda x: Color.BOLD+x+Color.ENDC
G = lambda x: Color.GREEN+x+Color.ENDC
R = lambda x: Color.RED+x+Color.ENDC
GR = lambda x: Color.GRAY+x+Color.ENDC


# prints housekeeping info given hkparam_t struct
def displayHK(hk):
    assert type(hk) == hkparam_t
    RES = lambda x: "Unknown reset" if x == 0 else "Dedicated WDT reset" if x == 1 else "I2C WDT reset" if x == 2 else "Hard reset" if x == 3 else "Soft reset" if x == 4 else "Stack overflow reset" if x == 5 else "Timer overflow reset" if x == 6 else "Brownout or power-on reset" if x == 7 else "Internal WDT reset" if x == 8 else "ERROR"
    mv = lambda x: x+"mV"
    ma = lambda x: x+"mA"
    degc = lambda x: x+"degC"
    mult = lambda x: lambda y: x*y
    add = lambda x: lambda y: x+y
    substr = lambda x: lambda y: y[x:]
    print("***************-HOUSEKEEPING-***************" >>_>> G)
    print(GR("Photo-voltaic inputs:        ")+"1-%s 2-%s 3-%s" %                                (hk.pv[0] >>_>> str >>_>> mv >>_>> R, 
                                                                                                 hk.pv[1] >>_>> str >>_>> mv >>_>> R, 
                                                                                                 hk.pv[2] >>_>> str >>_>> mv >>_>> R))

    print(GR("Total photo current:         ")+"%s" %                                            (hk.pc >>_>> str >>_>> ma >>_>> R))
    print(GR("Battery voltage:             ")+"%s" %                                            (hk.bv >>_>> str >>_>> mv >>_>> R))
    print(GR("Total system current:        ")+"%s" %                                            (hk.sc >>_>> str >>_>> ma >>_>> R))

    print(GR("Temp of boost converters:    ")+"1-%s 2-%s 3-%s batt-%s" %                        (hk.temp[0] >>_>> str >>_>> degc >>_>> R,
                                                                                                 hk.temp[1] >>_>> str >>_>> degc >>_>> R,
                                                                                                 hk.temp[2] >>_>> str >>_>> degc >>_>> R,
                                                                                                 hk.temp[3] >>_>> str >>_>> degc >>_>> R))

    print(GR("External batt temp:          ")+"1-%s 2-%s" %                                     (hk.batt_temp[0] >>_>> str >>_>> degc >>_>> R,
                                                                                                 hk.batt_temp[0] >>_>> str >>_>> degc >>_>> R))

    print(GR("Latchups:                    ")+"1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s]" %     (hk.latchup[0] >>_>> str >>_>> R,             
                                                                                                 hk.latchup[1] >>_>> str >>_>> R,
                                                                                                 hk.latchup[2] >>_>> str >>_>> R,
                                                                                                 hk.latchup[3] >>_>> str >>_>> R,
                                                                                                 hk.latchup[4] >>_>> str >>_>> R,
                                                                                                 hk.latchup[5] >>_>> str >>_>> R))

    print(GR("Cause of last reset:         ")+"%s" %                                            (hk.reset >>_>> RES >>_>> R))
    print(GR("Number of reboots:           ")+"%s" %                                            (hk.bootcount >>_>> str >>_>> R))
    print(GR("Number of software errors:   ")+"%s" %                                            (hk.sw_errors >>_>> str >>_>> R))
    print(GR("PPT mode:                    ")+"%s" %                                            (hk.ppt_mode >>_>> str >>_>> R))
    print(GR("Channel output:              ")+"%s" %                                            ((hk.channel_status >>_>> bin >>_>> substr(2) >>_>> len >>_>> mult(-1) >>_>> add(8) >>_>> mult("0"))+(hk.channel_status >>_>> bin >>_>> str >>_>> substr(2))) >>_>> R)
                                                                                                 

# prints config info given eps_config_t struct
def displayConfig(conf):
    assert type(conf) == eps_config_t
    pptmode = lambda x: "AUTO[1]" if x == 1 else "FIXED[2]" if x == 2 else "ERROR"
    battheatermode = lambda x: "MANUAL[0]" if x == 0 else "AUTO[1]" if x == 1 else "ERROR"
    mv = lambda x: x+"mV"
    ma = lambda x: x+"mA"
    degc = lambda x: x+"degC"
    sec = lambda x: x+"s"
    print("***************-CONFIG-***************" >>_>> G)
    print(GR("PPT mode:                    ")+"%s" %                                                      (conf.ppt_mode >>_>> pptmode >>_>> R))
    print(GR("Battheater mode:             ")+"%s" %                                                      (conf.battheater_mode >>_>> battheatermode >>_>> R))
    print(GR("Battheater low:              ")+"%s" %                                                      (conf.battheater_low >>_>> str >>_>> degc >>_>> R))
    print(GR("Battheater high:             ")+"%s" %                                                      (conf.battheater_high >>_>> str >>_>> degc >>_>> R))

    print(GR("Nominal mode output value:   ")+"1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]" % (conf.output_normal_value[0] >>_>> str >>_>> R,
                                                                                                         conf.output_normal_value[1] >>_>> str >>_>> R,
                                                                                                         conf.output_normal_value[2] >>_>> str >>_>> R,
                                                                                                         conf.output_normal_value[3] >>_>> str >>_>> R, 
                                                                                                         conf.output_normal_value[4] >>_>> str >>_>> R,
                                                                                                         conf.output_normal_value[5] >>_>> str >>_>> R,
                                                                                                         conf.output_normal_value[6] >>_>> str >>_>> R,
                                                                                                         conf.output_normal_value[7] >>_>> str >>_>> R))

    print(GR("Safe mode output value:      ")+"1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]" % (conf.output_safe_value[0] >>_>> str >>_>> R,
                                                                                                         conf.output_safe_value[1] >>_>> str >>_>> R,
                                                                                                         conf.output_safe_value[2] >>_>> str >>_>> R,
                                                                                                         conf.output_safe_value[3] >>_>> str >>_>> R, 
                                                                                                         conf.output_safe_value[4] >>_>> str >>_>> R,
                                                                                                         conf.output_safe_value[5] >>_>> str >>_>> R,
                                                                                                         conf.output_safe_value[6] >>_>> str >>_>> R,
                                                                                                         conf.output_safe_value[7] >>_>> str >>_>> R))

    print(GR("Output initial on:           ")+"1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]" % (conf.output_initial_on_delay[0] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[1] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[2] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[3] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[4] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[5] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[6] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_on_delay[7] >>_>> str >>_>> sec >>_>> R))

    print(GR("Output initial off:          ")+"1-[%s] 2-[%s] 3-[%s] 4-[%s] 5-[%s] 6-[%s] 7-[%s] 8-[%s]" % (conf.output_initial_off_delay[0] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[1] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[2] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[3] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[4] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[5] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[6] >>_>> str >>_>> sec >>_>> R,
                                                                                                         conf.output_initial_off_delay[7] >>_>> str >>_>> sec >>_>> R))

    print(GR("PPT point for boost conv:    ")+"1-%s 2-%s 3-%s" %                                          (conf.vboost[0] >>_>> str >>_>> mv >>_>> R,
                                                                                                         conf.vboost[1] >>_>> str >>_>> mv >>_>> R,
                                                                                                         conf.vboost[2] >>_>> str >>_>> mv >>_>> R))

# prints config2 info given eps_config2_t struct
def displayConfig2(conf):
    assert type(conf) == eps_config2_t
    mv = lambda x: x+"mV"
    print("***************-CONFIG2-***************" >>_>> G)
    print(GR("Batt Max Voltage:            ")+"%s" % (conf.batt_maxvoltage >>_>> str >>_>> mv >>_>> R))
    print(GR("Batt Safe Voltage:           ")+"%s" % (conf.batt_safevoltage >>_>> str >>_>> mv >>_>> R))
    print(GR("Batt Critical Voltage:       ")+"%s" % (conf.batt_criticalvoltage >>_>> str >>_>> mv >>_>> R))
    print(GR("Batt Normal Voltage:         ")+"%s" % (conf.batt_normalvoltage >>_>> str >>_>> mv >>_>> R))

# ----------------------------------------------CONSTANTS
CONFIG_DEFAULT = eps_config_t()
CONFIG_DEFAULT.ppt_mode = 2
CONFIG_DEFAULT.battheater_high = 5
CONFIG_DEFAULT.vboost[0] = 3700
CONFIG_DEFAULT.vboost[1] = 3700
CONFIG_DEFAULT.vboost[2] = 3700
