from flight_modes.flight_mode import FlightMode
from utils.log import get_log
from drivers.gom import Gomspace
from utils.constants import ENTER_LOW_BATTERY_MODE_THRESHOLD, FMEnum

logger = get_log()


class NormalMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)
        logger.info("Now in normal mode...")
        self.battery = None   # placeholder until it's set

    def run_mode(self):
        logger.info("Checking battery...")
        self.check_battery()
        logger.info("Checking sensors...")
        self.check_sensors()
        logger.info("Now running op nav...")
        self.run_opnav()
        logger.info("Now downlinking data...")
        self.downlink()

    # TODO
    def check_sensors(self):
        pass

    # TODO
    def check_battery(self):
        self.battery = Gomspace.read_battery_percentage
        logger.info("Battery percentage (btw 0 and 1) is: ", self.battery)
        if self.battery < ENTER_LOW_BATTERY_MODE_THRESHOLD:
            #TODO
            # enter low battery mode
            pass

    # TODO
    def run_opnav(self):
        # change the flight mode to opnav mode
        # feed in an enum of normal mode so it knows to go back
        pass

    # TODO
    def downlink(self):
        pass