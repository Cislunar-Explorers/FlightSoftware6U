from flight_modes.flight_mode import FlightMode
from utils.log import get_log

logger = get_log()


class NormalMode(FlightMode):
    def __init__(self, parent):
        super().__init__(parent)
        logger.info("Now in normal mode...")
        self.run_mode()

    def run_mode(self):
        logger.info("Checking sensors...")
        self.check_sensors()
        logger.info("Checking battery...")
        self.check_battery()
        logger.info("Now running op nav...")
        self.run_opnav()
        logger.info("Now downlinking data...")
        self.downlink()
        logger.info("Waiting for next interval to run normal mode")

    # TODO
    def check_sensors(self):
        pass

    # TODO
    def check_battery(self):
        pass

    # TODO
    def run_opnav(self):
        # change the flight mode to opnav mode
        # feed in an enum of normal mode so it knows to go back
        pass

    # TODO
    def downlink(self):
        pass