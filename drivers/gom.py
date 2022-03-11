from enum import Enum
from drivers.device import Device
import drivers.power.loadswitch as ls

import drivers.power.power_controller as power_controller
import logging

from drivers.power.power_structs import eps_hk_t


class Hk(Enum):
    DEFAULT = "default"
    EPS = "eps"
    VI = "vi"
    OUT = "out"
    WDT = "wdt"
    BASIC = "basic"
    CONFIG = "config"
    CONFIG2 = "config2"


class Gomspace(Device):

    driver: power_controller.Power

    lna: ls.P31uLoadSwitch
    burnwire: ls.P31uLoadSwitch
    glowplug_1: ls.P31uLoadSwitch
    glowplug_2: ls.P31uLoadSwitch
    solenoid: ls.solenoid
    electrolyzers: ls.P31uLoadSwitch
    pa: ls.power_amplifier
    rf_tx: ls.rf_switch_tx
    rf_rx: ls.rf_switch_rx

    def __init__(self) -> None:
        super().__init__("GOM")

    def _connect_to_hardware(self):
        self.driver = power_controller.Power()

        # load switch definition
        self.lna = ls.lna(self.driver)
        self.burnwire = ls.burnwire(self.driver)
        self.glowplug_1 = ls.glowplug_1(self.driver)
        self.glowplug_2 = ls.glowplug_2(self.driver)
        self.solenoid = ls.solenoid(self.driver)
        self.electrolyzers = ls.electrolyzers(self.driver)
        self.power_amplifier = ls.power_amplifier(self.driver)
        self.rf_tx = ls.rf_switch_tx(self.driver)
        self.rf_rx = ls.rf_switch_rx(self.driver)
        # self.heater = ls.heater(self.driver)

        # cleaner implementation of the above, but no autocomplete
        # loadswitches: List[ls.LoadSwitch] = [ls.lna, ls.burnwire,
        #                                      ls.glowplug_1, ls.glowplug_2, ls.solenoid, ls.electrolyzers]
        # for switch in loadswitches:
        #     setattr(self, type(switch).__name__, switch(self.driver))

    def _collect_telem(self) -> eps_hk_t:
        return self.driver.get_hk_2()

    def collect_telem(self) -> eps_hk_t:
        return super().collect_telem()

    def tick_wdt(self):
        """Resets dedicated WDT"""
        if self.connected:
            return self.driver.reset_wdt()
        else:
            logging.warning("Can't pet watchdog - Not connected to gom hardware")

    def all_off(self):
        """Turns off all controllable outputs on the Gomspace"""
        logging.debug("Turning off all controllable outputs")
        self.power_amplifier.set(False)
        self.driver.set_output(0)

    def hard_reset(self, passcode):
        """Performs a hard reset of the P31u, including cycling permanent 5V and 3.3V and battery outputs"""
        logging.info("Performing hard reset soon with passcode %s", passcode)
        self.driver.hard_reset(are_you_sure=passcode)

    def display_all(self):
        """Prints Housekeeping, config, and config2 data"""
        logging.debug("Printing housekeeping, config and config2 data")
        self.driver.displayAll()

    def is_electrolyzing(self) -> bool:
        """Returns whether the electrolyzer loadswitch is on"""
        return self.electrolyzers.get_telem().state
