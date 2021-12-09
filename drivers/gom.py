from enum import Enum

import utils.parameters as params
from utils.constants import GomOutputs

import drivers.power.power_controller as power_controller
import logging


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
        self.pc = power_controller.Power()

    def tick_wdt(self):
        """Resets dedicated WDT"""
        return self.pc.reset_wdt()

    def get_health_data(self, level=Hk.DEFAULT.value):
        """Returns a struct containing housekeeping data.
        The level parameter specifies which command gets sent to the P31u and what data you get back.
        level must be either one of the following: \n
        ["default", "eps", "vi", "out", "wdt", "basic", "config", "config2"]\n
        If no argument is provided, returns the same as "default" \n
        Every option returns a different struct, the documentation for which can be found in power_structs.py or in
        the GomSpace NanoPower P31u manual"""

        hk_dict = {
            Hk.DEFAULT.value: self.pc.get_hk_1,
            Hk.EPS.value: self.pc.get_hk_2,
            Hk.VI.value: self.pc.get_hk_2_vi,
            Hk.OUT.value: self.pc.get_hk_out,
            Hk.WDT.value: self.pc.get_hk_wdt,
            Hk.BASIC.value: self.pc.get_hk_2_basic,
            Hk.CONFIG.value: self.pc.config_get,
            Hk.CONFIG2.value: self.pc.config2_get,
        }

        try:
            logging.debug("Getting health data %s from get_health_data" % level)
            return hk_dict[level.lower()]()
        except KeyError:
            logging.warning(
                "Invalid argument in get_health_data. Getting default health data"
            )
            return self.pc.get_hk_1()

    def set_output(self, channel, value, delay=0):
        """Sets a single controllable output either on or off.
        channel must be a string that corresponds to one of
        the outputs (see power_controller.py)
        value must be either 1 (on) or 0 (off)"""

        self.pc.set_single_output(channel, value, delay)

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        logging.debug("Turning off all controllable outputs")
        self.set_pa(False)
        self.pc.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        logging.info("Performing hard reset soon with passcode %s", passcode)
        self.pc.hard_reset(are_you_sure=passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        logging.debug("Printing housekeeping, config and config2 data")
        self.pc.displayAll()

    def solenoid(self, hold):
        """Spikes the solenoid at 12V for [ACS_SPIKE_DURATION] milliseconds, holds at 5V for [hold] milliseconds"""
        self.pc.solenoid_single_wave(hold)

    def glowplug(self, duration, delay=0):
        """Pulses the glowplug for [duration] milliseconds with after a delay of [delay] seconds"""
        self.pc.glowplug(duration, delay)

    def burnwire1(self, duration, delay=0):
        """Turns on burnwire 1 for [duration] seconds after [delay] seconds. Does a display_all half way through"""
        self.pc.burnwire1(duration, delay)

    def glowplug2(self, duration, delay=0):
        """Turns on glowplug 2 for [duration] milliseconds after [delay] seconds"""
        self.pc.glowplug2(duration, delay)

    def set_electrolysis(self, status: bool, delay=0):
        """Switches on if [status] is true, off otherwise, with a delay of [delay] seconds."""
        self.pc.electrolyzer(status, delay=delay)

    def lna(self, on: bool):
        """Turns the receiving amplifier on (True)/off (False)"""
        self.pc.comms_amplifier(on)

    def rf_receiving_switch(self, receive: bool):
        """Tells receiving side of RF switch to either receive or transmit"""
        self.pc.rf_receiving_switch(receive)

    def rf_transmitting_switch(self, receive: bool):
        """Tells transmitting side of RF switch to either receive or transmit"""
        self.pc.rf_transmitting_switch(receive)

    def set_pa(self, on: bool):
        """Turns on/off the power circuit for the PA"""
        self.pc.set_pa(on)

    def is_electrolyzing(self) -> bool:
        """Returns status of electrolyzer"""
        output_struct = self.get_health_data(level=Hk.OUT.value)
        electrolyzer_on = output_struct.output[GomOutputs.electrolyzer.value]
        logging.info(f"Electrolyzer is {electrolyzer_on}")
        return bool(electrolyzer_on)

    def read_battery_percentage(self):
        """DEPRECATED. Use self.get_health_data(level=Hk.EPS.value).vbatt instead"""
        battery_data = self.get_health_data(level=Hk.VI.value)
        battery_voltage = battery_data.vbatt
        vmin = params.GOM_VOLTAGE_MIN
        vmax = params.GOM_VOLTAGE_MAX
        return (battery_voltage - vmin) / (vmax - vmin)
