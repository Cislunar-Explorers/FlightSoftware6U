from FlightSoftware.drivers.power.power_controller import *
import FlightSoftware.utils.log

def test_burnwire():    # input (duration, delay)
    pwr = Power()

    print("Test 1: ")
    pwr.burnwire(10, 0)
    print("Test 2: ")
    pwr.burnwire(2, 2)
    print("Test 3: ")
    pwr.burnwire(0, 0)
    print("Test 4: ")
    pwr.burnwire(0, 3)
