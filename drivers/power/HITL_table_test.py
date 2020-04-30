# Commands we want to test on the HITL table in SP2020

from power_controller import *
import time

HITL_test = Power()

print("Turning off all outputs")
OUTPUTS = [OUT_1, OUT_2, OUT_3, OUT_4, OUT_5, OUT_6]
for i in range(0, 6):
    HITL_test.set_single_output(OUTPUTS[i], 0,0)

print(" --- TESTING  displayAll --- \n")
HITL_test.displayAll()

print("\nBeginning output testing in 5 seconds\n")
time.sleep(5)

# Turn every channel on then off sequentially using set_single_output
print("\n --- TESTING OUPUTS --- \n")
for i in range(0, 6):
    current_output = OUTPUTS[i]
    print(" ### TESTING OUT_" + str(i) + " ###\n")
    HITL_test.set_single_output(current_output, 1, 0)  # Turns on channel
    time.sleep(1)  # wait one second
    HK_data = HITL_test.get_hk_1()  # get the housekeeping data
    HITL_test.set_single_output(current_output, 0, 0)  # Turn off channel
    print("OUT_" + str(i) + " System Current: " + str(HK_data.cursys))
    print("OUT_" + str(i) + " Battery Voltage: " + str(HK_data.vbatt))
    print("\n")
    time.sleep(5)

# Test the component-functions
# test burnwire:
# TODO: Check with Aaron (either one) about software requirements (i.e. what data the component functions should return)
print("Testing component functions in 5 seconds")
print("\n--- TESTING COMPONENT FUNCTIONS --- \n")
time.sleep(5)
print("Testing burnwire:")
print("You should see HITL outputs 9 and 10 light up")
HITL_test.burnwire(1)
time.sleep(1)

print("Testing Glowplug")
print("You should see output 11 light up")
HITL_test.glowplug(1)
time.sleep(1)

print("Testing Solenoid")
print("You should see HITL output 12 light up")
HITL_test.solenoid(10,990)
time.sleep(1)

print("Testing Electrolyzer")
print("You should see HITL output 13 light up for 10 seconds")
HITL_test.electrolyzer(True)
time.sleep(10)
HITL_test.electrolyzer(False)

print("\nComponent function testing done")
time.sleep(2)
print("\n--- Testing WDTs ---\n")
time.sleep(1)

# get wdt data
WDT_data = HITL_test.get_hk_wdt()
print("Initial WDT data:")
print("I2C Time left: " + str(WDT_data.wdt_i2c_time_left))
print("GND Time left: " + str(WDT_data.wdt_gnd_time_left))
print("CSP Pings left: " + str(WDT_data.wdt_csp_pings_left))
print("I2C Reboots: " + str(WDT_data.counter_wdt_i2c))
print("GND Reboots: " + str(WDT_data.counter_wdt_gnd))
print("CPS Reboots: " + str(WDT_data.counter_wdt_csp))

time.sleep(5)
# test i2c wdt
HITL_test.ping(1)
WDT_data_i2c_test = HITL_test.get_hk_wdt()
print("\nWDT data after I2C ping")
print("I2C Time left: " + str(WDT_data_i2c_test.wdt_i2c_time_left))
print("GND Time left: " + str(WDT_data_i2c_test.wdt_gnd_time_left))
print("CSP Pings left: " + str(WDT_data_i2c_test.wdt_csp_pings_left))
print("I2C Reboots: " + str(WDT_data_i2c_test.counter_wdt_i2c))
print("GND Reboots: " + str(WDT_data_i2c_test.counter_wdt_gnd))
print("CPS Reboots: " + str(WDT_data_i2c_test.counter_wdt_csp))

time.sleep(5)
# reset ground wdt
HITL_test.reset_wdt()

# see if it worked
WDT_data_ground_test = HITL_test.get_hk_wdt()
print("\nWDT data after Ground timer reset")
print("I2C Time left: " + str(WDT_data_ground_test.wdt_i2c_time_left))
print("GND Time left: " + str(WDT_data_ground_test.wdt_gnd_time_left))
print("CSP Pings left: " + str(WDT_data_ground_test.wdt_csp_pings_left))
print("I2C Reboots: " + str(WDT_data_ground_test.counter_wdt_i2c))
print("GND Reboots: " + str(WDT_data_ground_test.counter_wdt_gnd))
print("CPS Reboots: " + str(WDT_data_ground_test.counter_wdt_csp))

print("WDT Testing Done.")
