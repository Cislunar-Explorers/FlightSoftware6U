# Commands we want to test on the HITL table in SP2020

import drivers.power.power_controller as pc
import logging
import time

from fsw_utils.constants import GomOutputs

HITL_test = pc.Power()

logging.debug("Turning off all outputs")
for output in GomOutputs:
    HITL_test.set_single_output(output, 0, 0)

logging.debug(" --- TESTING  displayAll --- \n")
HITL_test.displayAll()

WDT_pre_data = HITL_test.get_hk_wdt()
logging.debug("Pre-Test WDT data:")
logging.debug("I2C Time left: " + str(WDT_pre_data.wdt_i2c_time_left))
logging.debug("GND Time left: " + str(WDT_pre_data.wdt_gnd_time_left))
logging.debug("CSP Pings left: " + str(WDT_pre_data.wdt_csp_pings_left))
logging.debug("I2C Reboots: " + str(WDT_pre_data.counter_wdt_i2c))
logging.debug("GND Reboots: " + str(WDT_pre_data.counter_wdt_gnd))
logging.debug("CPS Reboots: " + str(WDT_pre_data.counter_wdt_csp))

logging.info("\nBeginning output testing in 5 seconds\n")
time.sleep(5)

# Turn every channel on then off sequentially using set_single_output
logging.debug("\n --- TESTING OUPUTS --- \n")
for output in GomOutputs:
    logging.debug(f" ### TESTING OUT_{output.value} ###\n")
    HITL_test.set_single_output(output, 1, 0)  # Turns on channel
    time.sleep(1)  # wait one second
    HK_data = HITL_test.get_hk_2()  # get the housekeeping data
    HITL_test.set_single_output(output, 0, 0)  # Turn off channel
    logging.debug(f"OUT_{output.value} System Current: " + str(HK_data.cursys))
    logging.debug(f"OUT_{output.value} Battery Voltage: " + str(HK_data.vbatt))
    logging.debug("\n")
    time.sleep(5)

# Test the component-functions
# test burnwire:
# TODO: Check with Aaron (either one) about software requirements (i.e. what data the component fxns should return)
logging.debug("Testing component functions in 5 seconds")
logging.debug("\n--- TESTING COMPONENT FUNCTIONS --- \n")
time.sleep(5)
logging.debug("Testing burnwire:")
logging.debug("You should see HITL outputs 9 and 10 light up")
HITL_test.burnwire1(1)
time.sleep(1)

logging.debug("Testing Glowplug")
logging.debug("You should see output 11 light up")
HITL_test.glowplug(1)
time.sleep(1)

logging.debug("Testing Solenoid")
logging.debug("You should see HITL output 12 light up")
HITL_test.solenoid(10, 990)
time.sleep(1)

logging.debug("Testing Electrolyzer")
logging.debug("You should see HITL output 13 light up for 10 seconds")
HITL_test.set_single_output(GomOutputs.electrolyzer, 1, 0)
time.sleep(10)
HITL_test.set_single_output(GomOutputs.electrolyzer, 0, 0)

logging.debug("\nComponent function testing done")
time.sleep(2)
logging.debug("\n--- Testing WDTs ---\n")
time.sleep(1)

# get wdt data
WDT_data = HITL_test.get_hk_wdt()
logging.debug("Initial post-Test WDT data:")
logging.debug("I2C Time left: " + str(WDT_data.wdt_i2c_time_left))
logging.debug("GND Time left: " + str(WDT_data.wdt_gnd_time_left))
logging.debug("CSP Pings left: " + str(WDT_data.wdt_csp_pings_left))
logging.debug("I2C Reboots: " + str(WDT_data.counter_wdt_i2c))
logging.debug("GND Reboots: " + str(WDT_data.counter_wdt_gnd))
logging.debug("CPS Reboots: " + str(WDT_data.counter_wdt_csp))

time.sleep(5)
# test i2c wdt
HITL_test.ping(1)
WDT_data_i2c_test = HITL_test.get_hk_wdt()
logging.debug("\nWDT data after I2C ping")
logging.debug("I2C Time left: " + str(WDT_data_i2c_test.wdt_i2c_time_left))
logging.debug("GND Time left: " + str(WDT_data_i2c_test.wdt_gnd_time_left))
logging.debug("CSP Pings left: " + str(WDT_data_i2c_test.wdt_csp_pings_left))
logging.debug("I2C Reboots: " + str(WDT_data_i2c_test.counter_wdt_i2c))
logging.debug("GND Reboots: " + str(WDT_data_i2c_test.counter_wdt_gnd))
logging.debug("CPS Reboots: " + str(WDT_data_i2c_test.counter_wdt_csp))

time.sleep(5)
# reset ground wdt
HITL_test.reset_wdt()

# see if it worked
WDT_data_ground_test = HITL_test.get_hk_wdt()
logging.debug("\nWDT data after Ground timer reset")
logging.debug("I2C Time left: " + str(WDT_data_ground_test.wdt_i2c_time_left))
logging.debug("GND Time left: " + str(WDT_data_ground_test.wdt_gnd_time_left))
logging.debug("CSP Pings left: " + str(WDT_data_ground_test.wdt_csp_pings_left))
logging.debug("I2C Reboots: " + str(WDT_data_ground_test.counter_wdt_i2c))
logging.debug("GND Reboots: " + str(WDT_data_ground_test.counter_wdt_gnd))
logging.debug("CPS Reboots: " + str(WDT_data_ground_test.counter_wdt_csp))

logging.debug("WDT Testing Done.")
