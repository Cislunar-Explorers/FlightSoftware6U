import drivers.power.power_controller as pc
import drivers.power.power_structs as ps
from enum import Enum
from utils.constants import GomOutputs, GOM_VOLTAGE_MAX, GOM_VOLTAGE_MIN

logger = ps.gom_logger


class Hk(Enum):
    DEFAULT = "default"
    EPS = "eps"
    VI = "vi"
    OUT = "out"
    WDT = "wdt"
    BASIC = "basic"
    CONFIG = "config"
    CONFIG2 = "config2"


class Gomspace:
    def __init__(self):
        self.gom = pc.Power()

    def tick_wdt(self):
        """Resets dedicated WDT"""
        return self.gom.reset_wdt()

    def get_health_data(self, level=Hk.DEFAULT.value):
        """Returns a struct containing housekeeping data.
            The level parameter specifies which command gets sent to the P31u and what data you get back.
            level must be either one of the following: \n
            ["default", "eps", "vi", "out", "wdt", "basic", "config", "config2"]\n
            If no argument is provided, returns the same as "default" \n
            Every option returns a different struct, the documentation for which can be found in power_structs.py or in
            the GomSpace NanoPower P31u manual"""

        hk_dict = {
            Hk.DEFAULT.value: self.gom.get_hk_1,
            Hk.EPS.value: self.gom.get_hk_2,
            Hk.VI.value: self.gom.get_hk_2_vi,
            Hk.OUT.value: self.gom.get_hk_out,
            Hk.WDT.value: self.gom.get_hk_wdt,
            Hk.BASIC.value: self.gom.get_hk_2_basic,
            Hk.CONFIG.value: self.gom.config_get,
            Hk.CONFIG2.value: self.gom.config2_get,
        }

        try:
            logger.debug("Getting health data %s from get_health_data" % level)
            return hk_dict[level.lower()]()
        except KeyError:
            logger.warning(
                "Invalid argument in get_health_data. Getting default health data"
            )
            return self.gom.get_hk_1()

    def set_output(self, channel, value, delay=0):
        """Sets a single controllable output either on or off.
            channel must be a string that corresponds to one of
            the outputs (see power_controller.py)
            value must be either 1 (on) or 0 (off)"""

        self.gom.set_single_output(channel, value, delay)

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        logger.debug("Turning off all controllable outputs")
        self.gom.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        logger.info("Performing hard reset soon with passcode %s", passcode)
        self.gom.hard_reset(are_you_sure=passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        logger.debug("Printing housekeeping, config and config2 data")
        self.gom.displayAll()

    def solenoid(self, spike, hold, delay=0):
        """Spikes the solenoid at 20V for [spike] milliseconds, holds at 5V for [hold] milliseconds"""
        self.gom.solenoid(spike, hold, delay)

    def glowplug(self, duration, delay=0):
        """Pulses the glowplug for [duration] milliseconds with after a delay of [delay] seconds"""
        self.gom.glowplug(duration, delay)

    def burnwire1(self, duration, delay=0):
        """Turns on burnwire 1 for [duration] seconds after [delay] seconds. Does a display_all half way through"""
        self.gom.burnwire1(duration, delay)

    def glowplug2(self, duration, delay=0):
        """Turns on burnwire 2 for [duration] seconds after [delay] seconds. Does a display_all half way through"""
        self.gom.burnwire2(duration, delay)

    def set_electrolysis(self, status: bool, delay=0):
        """Switches on if [status] is true, off otherwise, with a delay of [delay] seconds."""
        self.gom.electrolyzer(status, delay=delay)

    def is_electrolyzing(self):
        """Returns status of electrolyzer"""
        output_struct = self.get_health_data(level=Hk.OUT.value)
        return bool(output_struct.output[GomOutputs.electrolyzer.value])

    def read_battery_percentage(self):
        battery_data = self.get_health_data(level=Hk.VI.value)
        battery_voltage = battery_data.vbatt
        return (battery_voltage - GOM_VOLTAGE_MIN) / (
                GOM_VOLTAGE_MAX - GOM_VOLTAGE_MIN)
