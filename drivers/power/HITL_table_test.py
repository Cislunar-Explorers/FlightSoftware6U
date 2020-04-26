# Commands we want to test on the HITL table in SP2020

from power_controller import *
from power_structs import *
import time

HITL_test = Power()

print("Turning off all outputs")
OUTPUTS = [OUT_1, OUT_2, OUT_3, OUT_4, OUT_5, OUT_6]
for i in range(1,6):
    HITL_test.set_single_output(OUTPUTS[i], 0,0)

print(" --- TESTING  displayAll --- \n")
HITL_test.displayAll()

print("\nBeginning output testing in 5 seconds\n")
time.sleep(5)

# Turn every channel on then off sequentially using set_single_output
print("\n --- TESTING OUPUTS --- \n")
for i in range(1, 6):
    current_output = OUTPUTS[i]
    print(" ### TESTING OUT_" + str(i) + " ###\n")
    HITL_test.set_single_output(current_output, 1, 0)  # Turns on channel
    time.sleep(1)  # wait one second
    HK_data = HITL_test.get_hk_1()  # get the housekeeping data
    HITL_test.set_single_output(current_output, 0, 0)  # Turn off channel
    displayHK(HK_data)
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