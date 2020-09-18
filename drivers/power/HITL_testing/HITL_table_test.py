# Commands we want to test on the HITL table in SP2020

import power_controller as pc
import power_structs as ps
import time

HITL_test = pc.Power()

ps.gom_logger.debug("Turning off all outputs")
OUTPUTS = ["comms", "burnwire_1", "burnwire_2", "glowplug", "solenoid", "electrolyzer"]
for i in range(0, 6):
    HITL_test.set_single_output(OUTPUTS[i], 0, 0)

ps.gom_logger.debug(" --- TESTING  displayAll --- \n")
HITL_test.displayAll()

WDT_pre_data = HITL_test.get_hk_wdt()
ps.gom_logger.debug("Pre-Test WDT data:")
ps.gom_logger.debug("I2C Time left: " + str(WDT_pre_data.wdt_i2c_time_left))
ps.gom_logger.debug("GND Time left: " + str(WDT_pre_data.wdt_gnd_time_left))
ps.gom_logger.debug("CSP Pings left: " + str(WDT_pre_data.wdt_csp_pings_left))
ps.gom_logger.debug("I2C Reboots: " + str(WDT_pre_data.counter_wdt_i2c))
ps.gom_logger.debug("GND Reboots: " + str(WDT_pre_data.counter_wdt_gnd))
ps.gom_logger.debug("CPS Reboots: " + str(WDT_pre_data.counter_wdt_csp))

ps.gom_logger.info("\nBeginning output testing in 5 seconds\n")
time.sleep(5)

# Turn every channel on then off sequentially using set_single_output
ps.gom_logger.debug("\n --- TESTING OUPUTS --- \n")
out_num = 0
for i in OUTPUTS:
    current_output = i
    ps.gom_logger.debug(" ### TESTING OUT_" + str(out_num) + " ###\n")
    HITL_test.set_single_output(current_output, 1, 0)  # Turns on channel
    time.sleep(1)  # wait one second
    HK_data = HITL_test.get_hk_2()  # get the housekeeping data
    HITL_test.set_single_output(current_output, 0, 0)  # Turn off channel
    ps.gom_logger.debug("OUT_" + str(out_num) + " System Current: " + str(HK_data.cursys))
    ps.gom_logger.debug("OUT_" + str(out_num) + " Battery Voltage: " + str(HK_data.vbatt))
    ps.gom_logger.debug("\n")
    out_num = out_num + 1
    time.sleep(5)

# Test the component-functions
# test burnwire:
# TODO: Check with Aaron (either one) about software requirements (i.e. what data the component functions should return)
ps.gom_logger.debug("Testing component functions in 5 seconds")
ps.gom_logger.debug("\n--- TESTING COMPONENT FUNCTIONS --- \n")
time.sleep(5)
ps.gom_logger.debug("Testing burnwire:")
ps.gom_logger.debug("You should see HITL outputs 9 and 10 light up")
HITL_test.burnwire(1)
time.sleep(1)

ps.gom_logger.debug("Testing Glowplug")
ps.gom_logger.debug("You should see output 11 light up")
HITL_test.glowplug(1)
time.sleep(1)

ps.gom_logger.debug("Testing Solenoid")
ps.gom_logger.debug("You should see HITL output 12 light up")
HITL_test.solenoid(10, 990)
time.sleep(1)

ps.gom_logger.debug("Testing Electrolyzer")
ps.gom_logger.debug("You should see HITL output 13 light up for 10 seconds")
HITL_test.electrolyzer(True)
time.sleep(10)
HITL_test.electrolyzer(False)

ps.gom_logger.debug("\nComponent function testing done")
time.sleep(2)
ps.gom_logger.debug("\n--- Testing WDTs ---\n")
time.sleep(1)

# get wdt data
WDT_data = HITL_test.get_hk_wdt()
ps.gom_logger.debug("Initial post-Test WDT data:")
ps.gom_logger.debug("I2C Time left: " + str(WDT_data.wdt_i2c_time_left))
ps.gom_logger.debug("GND Time left: " + str(WDT_data.wdt_gnd_time_left))
ps.gom_logger.debug("CSP Pings left: " + str(WDT_data.wdt_csp_pings_left))
ps.gom_logger.debug("I2C Reboots: " + str(WDT_data.counter_wdt_i2c))
ps.gom_logger.debug("GND Reboots: " + str(WDT_data.counter_wdt_gnd))
ps.gom_logger.debug("CPS Reboots: " + str(WDT_data.counter_wdt_csp))

time.sleep(5)
# test i2c wdt
HITL_test.ping(1)
WDT_data_i2c_test = HITL_test.get_hk_wdt()
ps.gom_logger.debug("\nWDT data after I2C ping")
ps.gom_logger.debug("I2C Time left: " + str(WDT_data_i2c_test.wdt_i2c_time_left))
ps.gom_logger.debug("GND Time left: " + str(WDT_data_i2c_test.wdt_gnd_time_left))
ps.gom_logger.debug("CSP Pings left: " + str(WDT_data_i2c_test.wdt_csp_pings_left))
ps.gom_logger.debug("I2C Reboots: " + str(WDT_data_i2c_test.counter_wdt_i2c))
ps.gom_logger.debug("GND Reboots: " + str(WDT_data_i2c_test.counter_wdt_gnd))
ps.gom_logger.debug("CPS Reboots: " + str(WDT_data_i2c_test.counter_wdt_csp))

time.sleep(5)
# reset ground wdt
HITL_test.reset_wdt()

# see if it worked
WDT_data_ground_test = HITL_test.get_hk_wdt()
ps.gom_logger.debug("\nWDT data after Ground timer reset")
ps.gom_logger.debug("I2C Time left: " + str(WDT_data_ground_test.wdt_i2c_time_left))
ps.gom_logger.debug("GND Time left: " + str(WDT_data_ground_test.wdt_gnd_time_left))
ps.gom_logger.debug("CSP Pings left: " + str(WDT_data_ground_test.wdt_csp_pings_left))
ps.gom_logger.debug("I2C Reboots: " + str(WDT_data_ground_test.counter_wdt_i2c))
ps.gom_logger.debug("GND Reboots: " + str(WDT_data_ground_test.counter_wdt_gnd))
ps.gom_logger.debug("CPS Reboots: " + str(WDT_data_ground_test.counter_wdt_csp))

ps.gom_logger.debug("WDT Testing Done.")
