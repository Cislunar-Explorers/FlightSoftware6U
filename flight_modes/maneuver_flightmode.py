from time import sleep, time

from .flight_mode import PauseBackgroundMode
from utils.log import get_log
from utils.parameter_utils import set_parameter
import utils.constants as consts
import utils.parameters as params

logger = get_log()
NO_ARGS = ([], 0)


class ManeuverMode(PauseBackgroundMode):
    flight_mode_id = consts.FMEnum.Maneuver.value
    command_codecs = {consts.ManeuverCommandEnum.Switch.value: NO_ARGS}
    command_arg_unpackers = {}
    glowplug1_valid = True
    glowplug2_valid = True

    def __init__(self, parent):
        super().__init__(parent)

    def get_pressure(self):
        return self._parent.adc.read_pressure()

    def valid_glowplug(self, current_pressure, prior_pressure):
        """Check pressure in place of acceleration for glowplug validation."""
        return current_pressure <= consts.PRESSURE_THRESHOLD and current_pressure - prior_pressure <= consts.PRESSURE_DELTA

    def update_state(self) -> int:
        if self.task_completed is True:
            logger.info("Maneuver complete. Exiting maneuver mode...")
            return consts.FMEnum.Normal.value
        return consts.NO_FM_CHANGE

    def run_mode(self):
        """Activate glowplugs until one works."""
        # TODO send info to ground??
        if params.SCHEDULED_BURN_TIME < time():
            logger.info("Maneuver time passed. Skipped.")
        else:
            # TODO what if SCHEDULED_BURN_TIME is too far into the future
            self.task_completed = False
            # sleeping for 5 fewer seconds than the delay for safety
            sleep(min(0, (params.SCHEDULED_BURN_TIME - time()) - 5))
            logger.info("Heating up glowplug to execute a maneuver...")

            prior_pressure = self.get_pressure()
            print(prior_pressure)
            if self.glowplug1_valid:
                self._parent.gom.glowplug(consts.GLOWPLUG_DURATION)
                sleep(consts.GLOW_WAIT_TIME)
                current_pressure = self.get_pressure()
                self.task_completed = self.valid_glowplug(current_pressure, prior_pressure)
                self.glowplug1_valid = self.task_completed
            if not self.glowplug1_valid and self.glowplug2_valid:
                self._parent.gom.glowplug2(consts.GLOWPLUG_DURATION)
                sleep(consts.GLOW_WAIT_TIME)
                current_pressure = self.get_pressure()
                self.task_completed = self.valid_glowplug(current_pressure, prior_pressure)
                self.glowplug2_valid = self.task_completed
            if not self.glowplug1_valid and not self.glowplug2_valid:
                # TODO ground msg?... do a final check?
                logger.info("All glowplugs failed.")
                self.task_completed = True

        # prepare next maneuver
        smallest_time_burn = -1
        while not self._parent.maneuver_queue.empty():
            smallest_time_burn = self._parent.maneuver_queue.get()
            if smallest_time_burn < time():
                # TODO send info to ground??
                smallest_time_burn = -1
                logger.info("Maneuver time passed. Skipped.")
            else:
                break

        set_parameter("SCHEDULED_BURN_TIME", smallest_time_burn, False)  # for testing
