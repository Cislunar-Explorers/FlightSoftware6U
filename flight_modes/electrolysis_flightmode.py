from .flight_mode import FlightMode
# TODO confirm path for ADC
from ..drivers.ADCDriver import ADC
from ..drivers.gom import *
from utils.constants import (
    LOW_CRACKING_PRESSURE,
    HIGH_CRACKING_PRESSURE,
    IDEAL_CRACKING_PRESSURE,
    FMEnum
)


# Electrolyze until pressure in the tank reaches IDEAL_CRACKING_PRESSURE
# TODO determine correct values based on testing
# TODO determine which of cracking pressures to change on
# NOTE: if we go with LOW_CRACKING_PRESSURE ensure that pressure
# doesn't dip below in interval between exiting
# electrolysis and hitting ignition
class ElectrolysisMode(FlightMode):

    flight_mode_id = FMEnum.Electrolysis.value

    def __init__(self, parent):
        super().__init__(parent)
        self.pressure_sensor = parent.pressure_sensor
        self.gom = parent.gom

    # Turn electrolysis on if propellant tank is below IDEAL_CRACKING_PRESSURE
    # If we have reached this cracking pressure, then turn electrolysis off
    # and set task_completed to True
    def run_mode(self):
    # TODO Make it that doesn't stay in mode for long time (can leave while it is on)
        curr_pressure = self.read_pressure()
        # Keep electrolysis running if below pressure limit
        if curr_pressure < IDEAL_CRACKING_PRESSURE:
            # Only send i2c command if already not running
            if not self.gom.is_electrolyzing():
                try:
                    self.gom.set_electrolysis(True, time_crit = 0)
                except PowerException as e:
                    print(e)
            self.run_mode()
        # Turn off after reaching optimal pressure
        else:
            try:
                self.gom.set_electrolysis(False, time_crit = 0)
            except PowerException as e:
                print(e)
            self.completed_task()

        # If electrolyzing is turned on, check to see if I should turn it off
        #curr_pressure = self.pressure_sensor.read_pressure()
        #if curr_pressure >= IDEAL_CRACKING_PRESSURE:
        #    try:
        #        self.gom.set_electrolysis(False)
        #    except PowerException as e:
        #        print(e)
        #    else:
        #        self.completed_task()
        #else:
        #    # Keep running mode until pressure condition satisfied
        #    try:
        #        self.gom.set_electrolysis(True)
        #    except PowerException as e:
        #        print(e)
        #    else:
        #        self.run_mode()