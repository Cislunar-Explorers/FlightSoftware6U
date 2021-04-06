# power_controller.py
#
# Desc: Drivers for the GomSpace NanoPower P31u
#
# Author: Daniel Kim (dsk252)
# Revisited by: Tobias Fischer (tmf97) and Stephen Zakoworotny (sjz38) in Spring 2020
#
# Date: 5/31/2020
#
# Status: Main functionality completed
#         Higher level functions completed
#         Some testing necessary

import pigpio
import drivers.power.power_structs as ps
from utils.constants import GomOutputs
import utils.parameters as params
from utils.exceptions import PowerException, PowerInputError, PowerReadError
from time import time, sleep

# power device address
POWER_ADDRESS = 0x02

# raspberry pi bus number
PI_BUS = 1

# allowed ranges
MAX_PV_VOLTAGE = 4000  # TODO: change later

# command registers
CMD_PING = 0x01
CMD_REBOOT = 0x04
CMD_GET_HK = 0x08
CMD_SET_OUTPUT = 0x09
CMD_SET_SINGLE_OUTPUT = 0x0A
CMD_SET_PV_VOLT = 0x0B
CMD_SET_PV_AUTO = 0x0C
CMD_SET_HEATER = 0x0D
CMD_RESET_COUNTERS = 0x0F
CMD_RESET_WDT = 0x10
CMD_CONFIG_CMD = 0x11
CMD_CONFIG_GET = 0x12
CMD_CONFIG_SET = 0x13
CMD_HARD_RESET = 0x14
CMD_CONFIG2_CMD = 0x15
CMD_CONFIG2_GET = 0x16
CMD_CONFIG2_SET = 0x17

# struct sizes in bytes
SIZE_HKPARAM_T = 44
SIZE_EPS_HK_T = 136
SIZE_EPS_HK_VI_T = 20
SIZE_EPS_HK_OUT_T = 64
SIZE_EPS_HK_WDT_T = 28
SIZE_EPS_HK_BASIC_T = 24
SIZE_EPS_CONFIG_T = 58
SIZE_EPS_CONFIG2_T = 20

# power output channels
OUT_1 = 0  # 5V
OUT_2 = 1  # 5V
OUT_3 = 2  # 5V
OUT_4 = 3  # 5V
OUT_5 = 4  # 5V
OUT_6 = 5  # 3.3V
OUT_HEATER = 6
OUT_SWITCH = 7

# Outputs on board:
#
#       H1
#     .-----.
#  47 | 0 3 | 48
#  49 | 1 4 | 50
#  51 | 2 5 | 52
#     '-----'
#

# GPIO outputs
RF_RX_EN = 19  # Physical pin 35
RF_TX_EN = 26  # Physical pin 37
PA_EN = 27  # Physical pin 13
OUT_PI_SOLENOID_ENABLE = 13  # Physical pin 33

# Precomputed solenoid command bytearrays
SOLENOID_ON_COMMAND = bytearray([CMD_SET_SINGLE_OUTPUT, OUT_5, 1, 0, 0])
SOLENOID_OFF_COMMAND = bytearray([CMD_SET_SINGLE_OUTPUT, OUT_5, 0, 0, 0])
SOLENOID_ON_LIST = [SOLENOID_ON_COMMAND]
SOLENOID_OFF_LIST = [SOLENOID_OFF_COMMAND]

_ = ps._


class Power:
    # initializes power object with bus [bus] and device address [addr]
    def __init__(self, bus=PI_BUS, addr=POWER_ADDRESS, flags=0):
        ps.gom_logger.debug(
            "Initializing Power object with bus %s, device address %s, and flags %s",
            bus,
            addr,
            flags,
        )
        self._pi = pigpio.pi()  # initialize pigpio object
        self._dev = self._pi.i2c_open(bus, addr, flags)  # initialize i2c device

        # initialize GPIO outputs
        self._pi.set_mode(RF_RX_EN, pigpio.OUTPUT)
        self._pi.set_mode(RF_TX_EN, pigpio.OUTPUT)
        self._pi.set_mode(PA_EN, pigpio.OUTPUT)
        self._pi.set_mode(OUT_PI_SOLENOID_ENABLE, pigpio.OUTPUT)

        self.solenoid_wave = []
        self.solenoid_wave_id = -1

        self.calculate_solenoid_wave()
        # initialize gom outputs (all off)
        self.set_output(0)

    # prints housekeeping/config/config2
    def displayAll(self):
        ps.displayHK(self.get_hk_1())
        ps.displayHk2(self.get_hk_2())
        ps.displayConfig(self.config_get())
        ps.displayConfig2(self.config2_get())

    # writes byte list [values] with command register [cmd]
    # raises: ValueError if cmd or values are not bytes
    # TODO: Implement PowerWriteError once we switch to new I2C library
    def write(self, cmd, values):
        ps.gom_logger.debug("Writing to register %s with byte list %s", cmd, values)
        self._pi.i2c_write_device(self._dev, bytearray([cmd] + values))

    # reads [bytes] number of bytes from the device and returns a bytearray
    def read(self, num_bytes):
        # first two read bytes -> [command][error code][data]
        ps.gom_logger.debug("Reading %s bytes from the device", num_bytes)
        (x, r) = self._pi.i2c_read_device(self._dev, num_bytes + 2)
        ps.gom_logger.debug("Read %s, and %s from device", x, r)
        if r[1] != 0:
            ps.gom_logger.error("Command %i failed with error code %i", r[0], r[1])
            raise PowerReadError(
                "Read Error: Command %i failed with error code %i" % (r[0], r[1])
            )
        else:
            return r[2:]

    # Not sure what value is in the below function, need to get cleared up
    # pings value
    # value [1 byte]
    # raises: ValueError if value is not a byte
    def ping(self, value):
        ps.gom_logger.debug("Pinging %s", value)
        self.write(CMD_PING, [value])
        return self.read(1)

    # reboot the system
    def reboot(self):
        ps.gom_logger.info("Performing Gomspace reboot")
        self.write(CMD_REBOOT, [0x80, 0x07, 0x80, 0x07])

    # returns hkparam_t struct
    def get_hk_1(self):
        ps.gom_logger.debug("Running get_hk_1")
        self.write(CMD_GET_HK, [])
        array = self.read(SIZE_HKPARAM_T)
        return ps.c_bytesToStruct(array, "hkparam_t")

    # returns eps_hk_t struct
    def get_hk_2(self):
        ps.gom_logger.debug("Running get_hk_2")
        self.write(CMD_GET_HK, [0x00])
        array = self.read(SIZE_EPS_HK_T)
        return ps.c_bytesToStruct(array, "eps_hk_t")

    # returns eps_hk_vi_t struct
    def get_hk_2_vi(self):
        ps.gom_logger.debug("Running get_hk_2_vi")
        self.write(CMD_GET_HK, [0x01])
        array = self.read(SIZE_EPS_HK_VI_T)
        return ps.c_bytesToStruct(array, "eps_hk_vi_t")

    # returns eps_hk_out_t struct
    def get_hk_out(self):
        ps.gom_logger.debug("Running get_hk_out")
        self.write(CMD_GET_HK, [0x02])
        array = self.read(SIZE_EPS_HK_OUT_T)
        return ps.c_bytesToStruct(array, "eps_hk_out_t")

    # returns eps_hk_wdt_t struct
    def get_hk_wdt(self):
        ps.gom_logger.debug("Running get_hk_wdt")
        self.write(CMD_GET_HK, [0x03])
        array = self.read(SIZE_EPS_HK_WDT_T)
        return ps.c_bytesToStruct(array, "eps_hk_wdt_t")

    # returns eps_hk_basic_t struct
    def get_hk_2_basic(self):
        ps.gom_logger.debug("Running get_hk_2_basic")
        self.write(CMD_GET_HK, [0x04])
        array = self.read(SIZE_EPS_HK_BASIC_T)
        return ps.c_bytesToStruct(array, "eps_hk_basic_t")

    # sets voltage output channels with bit mask:
    # byte [1 byte] -> [NC NC 3.3V3 3.3V2 3.3V1 5V3 5V2 5V1]
    # raises: ValueError if byte is not char
    def set_output(self, byte):
        ps.gom_logger.debug("Setting outputs to %s", byte)
        self.write(CMD_SET_OUTPUT, [byte])

    # sets a single output on or off:
    # channel [1 byte]  -> voltage = (0~5), BP4 heater = 6, BP4 switch = 7
    # value   [1 byte]  -> on = 1, off = 0
    # delay   [2 bytes] -> [seconds]
    # raises: AssertionError if channel or value are out of range
    # 		  AssertionError if delay is not a number
    def set_single_output(self, channel, value, delay):
        ps.gom_logger.debug(
            "Setting single channel %s output to value %s after delay %s",
            channel,
            value,
            delay,
        )

        if value not in [0, 1]:
            raise PowerInputError("Invalid Input: value must be 0 or 1")
        else:
            channel_num = GomOutputs[channel].value
            d = ps.toBytes(delay, 2)

            self.write(CMD_SET_SINGLE_OUTPUT, [channel_num, value] + list(d))

    # Set the voltage on the photo-voltaic inputs V1, V2, V3 in mV.
    # Takes effect when MODE = 2, See SET_PV_AUTO.
    # volt1~volt3 [2 bytes] -> value in mV
    # raises: PowerInputError if voltages are over the max pv voltage
    # Not tested
    def set_pv_volt(self, volt1, volt2, volt3):
        ps.gom_logger.debug("Setting PV voltage: %s, %s, %s", volt1, volt2, volt3)
        if volt1 > MAX_PV_VOLTAGE or volt2 > MAX_PV_VOLTAGE or volt3 > MAX_PV_VOLTAGE:
            ps.gom_logger.error("PV volt is attempting to be set above MAX_PV_VOLTAGE")
            raise PowerInputError(
                "Invalid Input: voltages must be below %i mV" % MAX_PV_VOLTAGE
            )
        else:
            v = bytearray(6)
            v[0:2] = ps.toBytes(volt1, 2)
            v[2:4] = ps.toBytes(volt2, 2)
            v[4:] = ps.toBytes(volt3, 2)
            self.write(CMD_SET_PV_VOLT, list(v))

    # Sets the solar cell power tracking mode:
    # mode [1 byte] ->
    # MODE = 0: Hardware default power point
    # MODE = 1: Maximum power point tracking
    # MODE = 2: Fixed software powerpoint, value set with SET_PV_VOLT, default 4V
    # raises: PowerInputError if mode is not 0, 1, or 2
    # Not tested
    def set_pv_auto(self, mode):
        ps.gom_logger.debug("Setting solar cell power tracking to mode %i", mode)
        if mode not in [0, 1, 2]:
            raise PowerInputError("Invalid Input: mode must be 0, 1 or 2")
        else:
            self.write(CMD_SET_PV_AUTO, [mode])

    # returns bytearray with heater modes
    # command   [1 byte]  -> 0 = Set heater on/off (toggle?)
    # heater    [1 byte]  -> 0 = BP4, 1 = Onboard, 2 = Both
    # mode      [1 byte]  -> 0 = OFF, 1 = ON
    # return    [2 bytes] -> heater modes
    # raises: PowerInputError if variables are not in correct range
    # Not tested
    def set_heater(self, command, heater, mode):
        ps.gom_logger.debug("Setting heater %i to mode %i", heater, mode)
        if command != 0 or heater not in [0, 1, 2] and mode not in [0, 1]:
            ps.gom_logger.error(
                "Invalid Input: command must be 0,heater must be 0, 1 or 2, and mode must be 0 or 1"
            )
            raise PowerInputError(
                """Invalid Input: command must be 0,
                heater must be 0, 1 or 2, and mode must be 0 or 1"""
            )
        else:
            self.write(CMD_SET_HEATER, [command, heater, mode])
            return self.read(2)

    # Not tested
    def get_heater(self):
        ps.gom_logger.debug("Running get_heater")
        self.write(CMD_SET_HEATER, [])
        return self.read(2)

    # resets the boot counter and WDT counters.
    # Not tested
    def reset_counters(self):
        ps.gom_logger.debug("Resetting boot and WDT counters")
        self.write(CMD_RESET_COUNTERS, [0x42])

    # resets (kicks) dedicated WDT.
    # Not tested
    def reset_wdt(self):
        ps.gom_logger.debug("Resetting WDT")
        self.write(CMD_RESET_WDT, [0x78])

    # Use this command to control the config system.
    # cmd [1 byte] -> cmd = 1: Restore default config
    # raises: PowerInputError if command is not 1
    def config_cmd(self, command):
        ps.gom_logger.debug("Running config_cmd: restoring default config")
        if command != 1:
            ps.gom_logger.error("Invalid input: command must be 1")
            raise PowerInputError("Invalid Input: command must be 1")
        else:
            self.write(CMD_CONFIG_CMD, [command])

    # returns eps_config_t structure
    def config_get(self) -> ps.eps_config_t:
        ps.gom_logger.debug("Getting current config")
        self.write(CMD_CONFIG_GET, [])
        return ps.c_bytesToStruct(self.read(SIZE_EPS_CONFIG_T), "eps_config_t")

    # takes eps_config_t struct and sets configuration
    # Input struct is of type eps_config_t
    def config_set(self, struct: ps.eps_config_t):
        ps.gom_logger.debug("Setting current config")
        array = struct >> _ >> ps.c_structToBytes >> _ >> ps.bytesToList
        self.write(CMD_CONFIG_SET, array)

    # Send this command to perform a hard reset of the P31u,
    # including cycling permanent 5V and 3.3V and battery outputs.
    # Not tested- issue running through HITL server
    def hard_reset(self, are_you_sure=False):
        if are_you_sure is True:
            ps.gom_logger.info("Hard reset Passcode correct: Performing hard reset")
            ps.gom_logger.critical("Cycling permanent 5V and 3.3V and battery outputs")
            self.write(CMD_HARD_RESET, [])
        else:
            ps.gom_logger.info("Hard reset Passcode incorrect: Aborting hard reset")

    # Use this command to control the config 2 system.
    # cmd [1 byte] -> cmd=1: Restore default config; cmd=2: Confirm current config
    # raises: PowerInputError if command is not a valid value
    def config2_cmd(self, command):
        ps.gom_logger.debug("Running config2_cmd with command=%s", command)
        if command not in [1, 2]:
            ps.gom_logger.error("Invalid Input: command must be 1 or 2")
            raise PowerInputError("Invalid Input: command must be 1 or 2")
        else:
            self.write(CMD_CONFIG2_CMD, [command])

    # Use this command to request the P31 config 2.
    # returns esp_config2_t struct
    def config2_get(self):
        ps.gom_logger.debug("Getting config2")
        self.write(CMD_CONFIG2_GET, [])
        return ps.c_bytesToStruct(self.read(SIZE_EPS_CONFIG2_T), "eps_config2_t")

    # Use this command to send config 2 to the P31
    # and save it (remember to also confirm it [using config2_cmd?])
    # Input struct is of type eps_config2_t
    def config2_set(self, struct):
        ps.gom_logger.debug("Setting config2")
        array = struct >> _ >> ps.c_structToBytes >> _ >> ps.bytesToList
        self.write(CMD_CONFIG2_SET, array)

    # Higher level functions -------------------------------------------------

    # output must be off before the function is called.
    # pulses output [output] high for some amount of
    # milliseconds [duration]; called after a delay of
    # [delay] seconds.
    def pulse(self, output, duration, delay=0):
        ps.gom_logger.debug("Pulsing Gom output %i on for %i ms after a %i sec delay")
        sleep(delay)
        self.set_single_output(output, 1, 0)
        sleep(duration * 0.001)
        self.set_single_output(output, 0, 0)

    # output must be off before the function is called
    # pulses high for duration amount of milliseconds
    def pulse_pi(self, output, duration, delay=0):
        ps.gom_logger.debug("Pulsing GPIO channel %i High for %i ms after a %i sec delay")
        sleep(delay)
        ps.gom_logger.debug("Setting GPIO channel %i HIGH", output)
        self._pi.write(output, 1)
        sleep(duration * 0.001)
        self._pi.write(output, 0)
        ps.gom_logger.debug("Set GPIO channel %i LOW", output)

    # switches on if [switch] is true, off otherwise, with a
    # delay of [delay] seconds.
    # Input switch is of type bool
    def electrolyzer(self, switch, delay=0):
        ps.gom_logger.debug(
            "Setting electrolyzer to mode %i after a delay of %i seconds", switch, delay
        )
        self.set_single_output("electrolyzer", int(switch), delay)

    # spikes the solenoid for [spike] milliseconds and
    # holds at 5v for [hold] milliseconds with a
    # delay of [delay] seconds.
    # output must be off before the function is called
    def solenoid(self, spike, hold):
        ps.gom_logger.warning("DEPRECATED FUNCTION Power().solenoid")
        ps.gom_logger.debug(
            "Spiking solenoid for %i ms, holding for %i ms, with a delay of %i sec",
            spike,
            hold,
        )

        self._pi.write(OUT_PI_SOLENOID_ENABLE, 1)  # enable vboost
        self.set_single_output("solenoid", 1, 0)
        sleep(0.001 * spike)
        self._pi.write(OUT_PI_SOLENOID_ENABLE, 0)  # disable vboost
        sleep(0.001 * hold)
        self.set_single_output("solenoid", 0, 0)

    # Experimental implementation of above functionality
    def solenoid_single_wave(self, hold):
        # self._pi.i2c_write_device(self._dev, SOLENOID_ON_COMMAND)  # consider replacing with set_output CMD
        pigpio._pigpio_command_ext(self._pi.sl, 57, self._dev, 0, 5, SOLENOID_ON_LIST)
        self._pi.wave_send_once(self.solenoid_wave_id)  # enables vboost - async
        sleep(hold)
        # self._pi.i2c_write_device(self._dev, SOLENOID_OFF_COMMAND)
        pigpio._pigpio_command_ext(self._pi.sl, 57, self._dev, 0, 5, SOLENOID_OFF_LIST)

    def calculate_solenoid_wave(self):
        self.solenoid_wave = []
        self.solenoid_wave.append(pigpio.pulse(1 << OUT_PI_SOLENOID_ENABLE, 0, params.ACS_SPIKE_DURATION * 1000))
        self.solenoid_wave.append(pigpio.pulse(0, 1 << OUT_PI_SOLENOID_ENABLE, 0))

        self._pi.wave_clear()
        self._pi.wave_add_generic(self.solenoid_wave)
        self.solenoid_wave_id = self._pi.wave_create()

    # pulses glowplug for [duration] milliseconds with
    # delay of [delay] seconds.
    # output must be off before the function is called
    def glowplug(self, duration, delay=0):
        ps.gom_logger.debug(
            "Pulsing glowplug for %i ms with a delay of %i sec", duration, delay
        )
        sleep(delay)
        self.set_single_output("glowplug", 1, 0)
        sleep(0.001 * duration)
        self.set_single_output("glowplug", 0, 0)

    # turns both burnwires on for [duration] seconds, with a
    # delay of [delay] seconds.
    def burnwire1(self, duration, delay=0):
        ps.gom_logger.debug(
            "Turning on burnwire 1 for %s seconds after a delay of %s sec",
            duration,
            delay,
        )
        sleep(delay)
        self.set_single_output("burnwire_1", 1, 0)
        sleep(duration / 2)
        self.displayAll()
        sleep(duration / 2)
        self.set_single_output("burnwire_1", 0, 0)

    # turns both burnwire 2 on for [duration] seconds, with a
    # delay of [delay] seconds.
    def glowplug2(self, duration, delay=0):
        ps.gom_logger.info(
            "Turning on glowplug 2 for %s seconds after a delay of %s sec",
            duration,
            delay,
        )

        sleep(delay)
        self.set_single_output("glowplug_2", 1, 0)
        sleep(duration * 0.001 / 2)
        self.displayAll()
        sleep(duration * 0.001 / 2)
        self.set_single_output("glowplug_2", 0, 0)

    # tell RF switch to either transmit or receive
    def rf_transmitting_switch(self, receive: bool = True):
        if receive:
            # Set transmitting side of RF switch to receive
            self._pi.write(RF_TX_EN, pigpio.LOW)
        else:
            # Set transmitting side of RF switch to transmit
            self._pi.write(RF_TX_EN, pigpio.HIGH)

    def rf_receiving_switch(self, receive: bool = True):
        if receive:
            # Set receiving RF switch to receive
            self._pi.write(RF_RX_EN, pigpio.HIGH)
        else:
            # Set receiving RF switch to transmit
            self._pi.write(RF_RX_EN, pigpio.LOW)

    def set_PA(self, on: bool):
        self._pi.write(PA_EN, int(on))

    # Toggles receiving comms amp on/off
    # Input on is either True (on) or False (off)
    def comms_amplifier(self, on: bool):
        self.set_single_output("comms", int(on), 0)

    def set_GPIO_low(self):
        self.rf_transmitting_switch()
        self.rf_receiving_switch()
        self._pi.write(OUT_PI_SOLENOID_ENABLE, 0)

    # Legacy stuff, may or may not be useful

    # def adjust_string(self, string, length):
    #     new = string
    #     if len(new) > length:
    #         new = new[:length]
    #     elif len(new) < length:
    #         new += " " * (length - len(new))
    #     return new

    # def nasa_demo(self):
    #     r = input("electrolyzers: ")
    #     self.electrolyzer(True)
    #     sleep(float(r))
    #     self.electrolyzer(False)
    #
    #     r = input("sparkplug:")
    #     self.sparkplug(3)
    #
    #     r = input("solenoid:")
    #     self.solenoid(10, 250)
    #
    #     r = input("burnwire:")
    #     self.burnwire(4)
    #
    #     r = input("sensor data:")
    #     self.display_sensors()

    # def chamber(self, name, t):
    #     text_file = open(name, "w")
    #     header = "time:" + str(t) + "\n"
    #     text_file.write(header)
    #     while True:
    #         try:
    #             hk = self.get_hk_1()
    #             nxt = (
    #                 "pv "
    #                 + str(hk.pv[0])
    #                 + ","
    #                 + str(hk.pv[1])
    #                 + ","
    #                 + str(hk.pv[2])
    #                 + "\n"
    #                 + "pc "
    #                 + str(hk.pc)
    #                 + "\n"
    #                 + "bv "
    #                 + str(hk.bv)
    #                 + "\n"
    #                 + "sc "
    #                 + str(hk.sc)
    #                 + "\n"
    #                 + "temp "
    #                 + str(hk.temp[0])
    #                 + ","
    #                 + str(hk.temp[1])
    #                 + ","
    #                 + str(hk.temp[2])
    #                 + ","
    #                 + str(hk.temp[3])
    #                 + "\n@\n"
    #             )
    #             print(nxt)
    #             text_file.write(nxt)
    #         except:
    #             print("recovered from error")
    #         sleep(t)
