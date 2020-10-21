# Testing all untested and new Gom-related commands

from drivers.gom import Gomspace, logger
from drivers.power.power_structs import displayHK, displayHk2, displayStruct
from time import sleep

test = Gomspace()

# testing all HK functions
hkdata = test.get_health_data()
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="eps")
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="vi")
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="out")
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="wdt")
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="basic")
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="config")
displayStruct(hkdata)
sleep(2)

hkdata = test.get_health_data(level="config2")
displayStruct(hkdata)
sleep(2)

# testing new burwire functions

test.burnwire1(5)

test.burnwire2(5)

sleep(3)

logger.debug("Is electrolyzing?: " + str(test.is_electrolyzing()))
sleep(1)

test.set_electrolysis(True)
sleep(1)
logger.debug("Is electrolyzing?: " + str(test.is_electrolyzing()))
sleep(1)
test.set_electrolysis(False)
sleep(1)
logger.debug("Is electrolyzing?: " + str(test.is_electrolyzing()))
sleep(1)

logger.debug("Battery %: " + str(test.read_battery_percentage()))
