from .flight_mode import FlightMode
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
        # If electrolyzing is turned on, check to see if I should turn it off
        curr_pressure = self.pressure_sensor.read_value()
        if curr_pressure >= IDEAL_CRACKING_PRESSURE:
            self.gom.set_electrolysis(False)
            self.completed_task()
        else:
            self.gom.set_electrolysis(True)