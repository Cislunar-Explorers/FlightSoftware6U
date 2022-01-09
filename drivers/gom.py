from enum import Enum
import drivers.power.loadswitch as ls

import utils.parameters as params

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
        self.driver = power_controller.Power()

        # load switch definition
        self.lna = ls.lna(self.driver)
        self.burnwire = ls.burnwire(self.driver)
        self.glowplug_1 = ls.glowplug_1(self.driver)
        self.glowplug_2 = ls.glowplug_2(self.driver)
        self.solenoid = ls.solenoid(self.driver)
        self.electrolyzers = ls.electrolyzers(self.driver)
        self.pa = ls.power_amplifier(self.driver)
        self.rf_tx = ls.rf_switch_tx(self.driver)
        self.rf_rx = ls.rf_switch_rx(self.driver)
        # self.heater = ls.heater(self.driver)

        # cleaner implementation of the above, but no autocomplete
        # loadswitches: List[ls.LoadSwitch] = [ls.lna, ls.burnwire,
        #                                      ls.glowplug_1, ls.glowplug_2, ls.solenoid, ls.electrolyzers]
        # for switch in loadswitches:
        #     setattr(self, type(switch).__name__, switch(self.driver))

    def tick_wdt(self):
        """Resets dedicated WDT"""
        return self.driver.reset_wdt()

    def get_health_data(self, level=Hk.DEFAULT.value):
        """Returns a struct containing housekeeping data.
        The level parameter specifies which command gets sent to the P31u and what data you get back.
        level must be either one of the following: \n
        ["default", "eps", "vi", "out", "wdt", "basic", "config", "config2"]\n
        If no argument is provided, returns the same as "default" \n
        Every option returns a different struct, the documentation for which can be found in power_structs.py or in
        the GomSpace NanoPower P31u manual"""

        hk_dict = {
            Hk.DEFAULT.value: self.driver.get_hk_1,
            Hk.EPS.value: self.driver.get_hk_2,
            Hk.VI.value: self.driver.get_hk_2_vi,
            Hk.OUT.value: self.driver.get_hk_out,
            Hk.WDT.value: self.driver.get_hk_wdt,
            Hk.BASIC.value: self.driver.get_hk_2_basic,
            Hk.CONFIG.value: self.driver.config_get,
            Hk.CONFIG2.value: self.driver.config2_get,
        }

        try:
            logging.debug("Getting health data %s from get_health_data" % level)
            return hk_dict[level.lower()]()
        except KeyError:
            logging.warning(
                "Invalid argument in get_health_data. Getting default health data"
            )
            return self.driver.get_hk_1()

    def set_output(self, channel, value, delay=0):
        """Sets a single controllable output either on or off.
        channel must be a string that corresponds to one of
        the outputs (see power_controller.py)
        value must be either 1 (on) or 0 (off)"""

        self.driver.set_single_output(channel, value, delay)

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        logging.debug("Turning off all controllable outputs")
        self.pa.set(False)
        self.driver.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        logging.info("Performing hard reset soon with passcode %s", passcode)
        self.driver.hard_reset(are_you_sure=passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        logging.debug("Printing housekeeping, config and config2 data")
        self.driver.displayAll()

    def read_battery_percentage(self):
        """DEPRECATED. Use self.get_health_data(level=Hk.EPS.value).vbatt instead"""
        battery_data = self.get_health_data(level=Hk.VI.value)
        battery_voltage = battery_data.vbatt
        vmin = params.GOM_VOLTAGE_MIN
        vmax = params.GOM_VOLTAGE_MAX
        return (battery_voltage - vmin) / (vmax - vmin)
