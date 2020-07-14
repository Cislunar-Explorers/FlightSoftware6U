from FlightSoftware.drivers.power.power_controller import *
from FlightSoftware.utils.log import *


def test_burnwire():    # input (duration, delay)
    pwr = Power()

    log.info('BurnWire Test: ')
    log.info(pwr.burnwire(10, 0))
    log.info(pwr.burnwire(2, 2))
    log.info(pwr.burnwire(0, 0))
    log.info(pwr.burnwire(0, 3))
