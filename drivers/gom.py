import power_controller as pc
from enum import Enum
import logging

logging.basicConfig(
    filename="gom.log",
    level=logging.DEBUG,
    format="[%(asctime)s]  %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S%z",  # ISO8601 timestamp
)


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
        logging.info("Initializing Gomspace object")
        self.gom = pc.Power()

    def tick_wdt(self):
        """Resets dedicated WDT"""
        logging.info("Resetting dedicated WDT")
        return self.gom.reset_wdt()

    def get_health_data(self, level=Hk.DEFAULT.value):
        """Returns a struct containing housekeeping data.
            The level parameter specifies which command gets sent to the P31u and what data you get back.
            level must be  one of the following: \n
            ["default", "eps", "vi", "out", "wdt", "basic", "config", "config2"]\n
            If no argument is provided, returns the same as "default" \n
            Every option returns a different struct, the documentation for which can be found in power_structs.py or in
            the GomSpace NanoPower P31u manual"""

        hk_dict = {
            Hk.DEFAULT.value: self.gom.get_hk_1(),
            Hk.EPS.value: self.gom.get_hk_2(),
            Hk.VI.value: self.gom.get_hk_2_vi(),
            Hk.OUT.value: self.gom.get_hk_out(),
            Hk.WDT.value: self.gom.get_hk_wdt(),
            Hk.BASIC.value: self.gom.get_hk_2_basic(),
            Hk.CONFIG.value: self.gom.config_get(),
            Hk.CONFIG2.value: self.gom.config2_get(),
        }

        try:
            logging.info("Getting health data %s from get_health_data" % level)
            return hk_dict[level]
        except KeyError:
            logging.warning(
                "Invalid argument in get_health_data. Getting default health data"
            )
            return hk_dict.get(Hk.DEFAULT.value)

    def set_output(self, channel, value, delay=0):
        """Sets a single controllable output either on or off.
            channel must be a string that corresponds to one of
            the outputs (see power_controller.py)
            value must be either 1 (on) or 0 (off)"""

        logging.info(
            "Setting output channel %s to value %s with delay %s", channel, value, delay
        )
        self.gom.set_single_output(channel, value, delay)

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        logging.info("Turning off all controllable outputs")
        self.gom.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        logging.info("Performing hard reset soon with passcode %s", passcode)
        self.gom.hard_reset(are_you_sure=passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        logging.info("Printing housekeeping, config and config2 data")
        self.gom.displayAll()

    def solenoid(self, spike, hold, delay=0):
        """Spikes the solenoid at 20V for [spike] milliseconds, holds at 5V for [hold] milliseconds"""
        logging.info(
            "Spiking solenoid for %s ms, holding for %s ms, after a delay of %s seconds",
            spike,
            hold,
            delay,
        )
        self.gom.solenoid(spike, hold, delay)
        self.electrolysis = False

    def glowplug(self, duration, delay=0):
        """Pulses the glowplug for [duration] milliseconds with after a delay of [delay] seconds"""
        logging.info(
            "Pulsing glowplug for %s ms after a delay of %s sec", duration, delay
        )
        self.gom.glowplug(duration, delay)

    def burnwires(self, duration, delay=0):
        """Turns on both burnwires for [duration] seconds after [delay] seconds. Does a display_all half way through"""
        logging.info(
            "Turning on both burnwires for %s seconds after a delay of %s sec",
            duration,
            delay,
        )
        self.gom.burnwire(duration, delay)

    def set_output(self, channel, value, delay=0):
        """Sets a single controllable output either on or off.
            channel must be a string that corresponds to one of
            the outputs (see power_controller.py)
            value must be either 1 (on) or 0 (off)"""
        self.gom.set_single_output(channel, value, delay)

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        self.gom.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        self.gom.hard_reset(passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        self.gom.displayAll()

    def solenoid(self, spike, hold, delay=0):
        """Spikes the solenoid at 20V for [spike] milliseconds, holds at 5V for [hold] milliseconds"""
        self.gom.solenoid(spike, hold, delay)

    def glowplug(self, duration, delay=0):
        """Pulses the glowplug for [duration] milliseconds with after a delay of [delay] seconds"""
        self.gom.glowplug(duration, delay)

    def burnwires(self, duration, delay=0):
        """Turns on both burnwires for [duration] seconds after [delay] seconds. Does a display_all half way through"""
        self.gom.burnwire(duration, delay)

    def set_electrolysis(self, status: bool, delay=0):
        """Switches on if [status] is true, off otherwise, with a delay of [delay] seconds."""
        self.electrolysis = status
        self.gom.electrolyzer(status, delay)

    def is_electrolyzing(self):
        """Returns status of electrolyzer"""
        return self.electrolysis

    # TODO
    def read_battery_percentage(self):
        return 0.7
