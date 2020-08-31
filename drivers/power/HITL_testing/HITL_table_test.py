# Commands we want to test on the HITL table in SP2020

from power_controller import *
import time


HITL_test = Power()

logger.debug("Turning off all outputs")
OUTPUTS = ["comms", "burnwire_1", "burnwire_2", "glowplug", "solenoid", "electrolyzer"]
for i in range(0, 6):
    HITL_test.set_single_output(OUTPUTS[i], 0, 0)

logger.debug(" --- TESTING  displayAll --- \n")
HITL_test.displayAll()

WDT_pre_data = HITL_test.get_hk_wdt()
logger.debug("Pre-Test WDT data:")
logger.debug("I2C Time left: " + str(WDT_pre_data.wdt_i2c_time_left))
logger.debug("GND Time left: " + str(WDT_pre_data.wdt_gnd_time_left))
logger.debug("CSP Pings left: " + str(WDT_pre_data.wdt_csp_pings_left))
logger.debug("I2C Reboots: " + str(WDT_pre_data.counter_wdt_i2c))
logger.debug("GND Reboots: " + str(WDT_pre_data.counter_wdt_gnd))
logger.debug("CPS Reboots: " + str(WDT_pre_data.counter_wdt_csp))

logger.info("\nBeginning output testing in 5 seconds\n")
time.sleep(5)

# Turn every channel on then off sequentially using set_single_output
logger.debug("\n --- TESTING OUPUTS --- \n")
out_num = 0
for i in Outputs:
    current_output = Outputs(i).name
    logger.debug(" ### TESTING OUT_" + str(out_num) + " ###\n")
    HITL_test.set_single_output(current_output, 1, 0)  # Turns on channel
    time.sleep(1)  # wait one second
    HK_data = HITL_test.get_hk_2()  # get the housekeeping data
    HITL_test.set_single_output(current_output, 0, 0)  # Turn off channel
    logger.debug("OUT_" + str(out_num) + " System Current: " + str(HK_data.cursys))
    logger.debug("OUT_" + str(out_num) + " Battery Voltage: " + str(HK_data.vbatt))
    logger.debug("\n")
    out_num = out_num + 1
    time.sleep(5)

# Test the component-functions
# test burnwire:
# TODO: Check with Aaron (either one) about software requirements (i.e. what data the component functions should return)
logger.debug("Testing component functions in 5 seconds")
logger.debug("\n--- TESTING COMPONENT FUNCTIONS --- \n")
time.sleep(5)
logger.debug("Testing burnwire:")
logger.debug("You should see HITL outputs 9 and 10 light up")
HITL_test.burnwire(1)
time.sleep(1)

logger.debug("Testing Glowplug")
logger.debug("You should see output 11 light up")
HITL_test.glowplug(1)
time.sleep(1)

logger.debug("Testing Solenoid")
logger.debug("You should see HITL output 12 light up")
HITL_test.solenoid(10, 990)
time.sleep(1)

logger.debug("Testing Electrolyzer")
logger.debug("You should see HITL output 13 light up for 10 seconds")
HITL_test.electrolyzer(True)
time.sleep(10)
HITL_test.electrolyzer(False)

logger.debug("\nComponent function testing done")
time.sleep(2)
logger.debug("\n--- Testing WDTs ---\n")
time.sleep(1)

# get wdt data
WDT_data = HITL_test.get_hk_wdt()
logger.debug("Initial post-Test WDT data:")
logger.debug("I2C Time left: " + str(WDT_data.wdt_i2c_time_left))
logger.debug("GND Time left: " + str(WDT_data.wdt_gnd_time_left))
logger.debug("CSP Pings left: " + str(WDT_data.wdt_csp_pings_left))
logger.debug("I2C Reboots: " + str(WDT_data.counter_wdt_i2c))
logger.debug("GND Reboots: " + str(WDT_data.counter_wdt_gnd))
logger.debug("CPS Reboots: " + str(WDT_data.counter_wdt_csp))

time.sleep(5)
# test i2c wdt
HITL_test.ping(1)
WDT_data_i2c_test = HITL_test.get_hk_wdt()
logger.debug("\nWDT data after I2C ping")
logger.debug("I2C Time left: " + str(WDT_data_i2c_test.wdt_i2c_time_left))
logger.debug("GND Time left: " + str(WDT_data_i2c_test.wdt_gnd_time_left))
logger.debug("CSP Pings left: " + str(WDT_data_i2c_test.wdt_csp_pings_left))
logger.debug("I2C Reboots: " + str(WDT_data_i2c_test.counter_wdt_i2c))
logger.debug("GND Reboots: " + str(WDT_data_i2c_test.counter_wdt_gnd))
logger.debug("CPS Reboots: " + str(WDT_data_i2c_test.counter_wdt_csp))

time.sleep(5)
# reset ground wdt
HITL_test.reset_wdt()

# see if it worked
WDT_data_ground_test = HITL_test.get_hk_wdt()
logger.debug("\nWDT data after Ground timer reset")
logger.debug("I2C Time left: " + str(WDT_data_ground_test.wdt_i2c_time_left))
logger.debug("GND Time left: " + str(WDT_data_ground_test.wdt_gnd_time_left))
logger.debug("CSP Pings left: " + str(WDT_data_ground_test.wdt_csp_pings_left))
logger.debug("I2C Reboots: " + str(WDT_data_ground_test.counter_wdt_i2c))
logger.debug("GND Reboots: " + str(WDT_data_ground_test.counter_wdt_gnd))
logger.debug("CPS Reboots: " + str(WDT_data_ground_test.counter_wdt_csp))

logger.debug("WDT Testing Done.")
