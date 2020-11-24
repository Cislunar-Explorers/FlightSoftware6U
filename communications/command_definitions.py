from main import MainSatelliteThread
from datetime import datetime
import utils.constants as constants
import os
import time


class CommandDefinitions:
    def __init__(self, parent: MainSatelliteThread):
        self.parent = parent
        self.bootup_commands = {1: self.split}
        self.restart_commands = {}
        self.normal_commands = {1: self.run_opnav,
                                2: self.change_attitude,
                                5: self.set_parameter,
                                6: self.gather_critical_telem,
                                7: self.gather_basic_telem,
                                8: self.gather_detailed_telem,
                                9: self.verification
                                }

        self.electrolysis_commands = {1: self.electrolysis,
                                      5: self.set_parameter,
                                      6: self.gather_critical_telem,
                                      7: self.gather_basic_telem,
                                      8: self.gather_detailed_telem
                                      }

        self.low_battery_commands = {1: self.return_to_normal,
                                     2: self.set_exit_lowbatt_threshold,
                                     5: self.set_parameter,
                                     6: self.gather_critical_telem,
                                     7: self.gather_basic_telem,
                                     8: self.gather_detailed_telem,
                                     }

        self.safety_commands = {1: self.return_to_normal,
                                # 2: Not Implemented/need clarification,
                                5: self.set_parameter,
                                6: self.gather_critical_telem,
                                7: self.gather_basic_telem,
                                8: self.gather_detailed_telem
                                }

        self.opnav_commands = {1: self.run_opnav,
                               2: self.set_opnav_interval
                               }

        self.maneuver_commands = {1: self.run_opnav,
                                  2: self.change_attitude,
                                  9: self.burn}

        self.sensor_commands = {}

        self.test_commands = {2: self.split,
                              3: self.run_opnav,
                              6: self.gom_outputs}

        self.comms_commands = {}

        self.command_commands = {1: self.set_parameter,
                                 2: self.set_system_clock,
                                 3: self.reboot_pi,
                                 4: self.reboot_gom,
                                 5: self.power_cycle,
                                 6: self.gom_outputs,
                                 7: self.gom_command,
                                 8: self.general_command,
                                 170: self.cease_comms}

        self.COMMAND_DICT = {
            0: self.bootup_commands,
            1: self.restart_commands,
            2: self.normal_commands,
            3: self.low_battery_commands,
            4: self.safety_commands,
            5: self.opnav_commands,
            6: self.maneuver_commands,
            7: self.sensor_commands,
            8: self.test_commands,
            9: self.comms_commands,
            10: self.command_commands
        }

    def split(self):
        # read gyro rate data before split
        self.parent.gom.burnwire2(constants.SPLIT_BURNWIRE_DURATION)
        # read gyro rotation rate data after split - need to downlink these to make sure of successful split

    def run_opnav(self):
        # self.parent.run_opnav
        raise NotImplementedError

    def set_parameter(self, obj, name, value):
        initial_value = getattr(obj, name)
        setattr(obj, name, value)
        changed_value = getattr(obj, name)
        self.parent.logger.info(f"Changed constant {name} from {initial_value} to {changed_value}")

        # TODO: implement "saving" and reading of parameters to a text file

    def set_exit_lowbatt_threshold(self, value):
        assert 0 < value < 1.0
        self.set_parameter(constants, "EXIT_LOW_BATTERY_MODE_THRESHOLD", value)

    def set_opnav_interval(self, value):
        assert value > 0
        self.set_parameter(constants, "OPNAV_INTERVAL", value)

    def change_attitude(self, theta, phi):
        raise NotImplementedError

    def gather_critical_telem(self):
        # here we want to only gather the most critical telemetry values so that we spend the least electricity
        # downlinking them (think about a low-power scenario where the most important thing is our power in and out)
        raise NotImplementedError

    def gather_basic_telem(self):
        # what's defined in section 3.6.1 of https://cornell.app.box.com/file/629596158344 would be a good packet
        raise NotImplementedError

    def gather_detailed_telem(self):
        # here we'd gather as much data about the satellite as possible
        raise NotImplementedError

    def verification(self):
        # Some verification defined by the NASA Cubequest challenge
        raise NotImplementedError

    def electrolysis(self, state: bool, delay: int):
        self.parent.gom.set_electrolysis(state, delay=delay)

    def burn(self, time, absolute: bool):
        if absolute:  # i.e. if we want to burn at a specific absolute time
            delay = time - datetime.now()
        else:  # if we want to burn exactly x seconds from receiving the command
            delay = time

        delay = max(0, delay)  # makes sure we don't have negative delays
        self.parent.gom.glowplug(constants.GLOWPLUG_DURATION, delay=delay)
        # TODO: need to make gom commands asynchronous (currently they make the whole satellite sleep for the delay
        #  instead of using the gom's delay option)

    def return_to_normal(self):
        self.parent.replace_flight_mode_by_id(constants.FMEnum.Normal.value)

    def reboot_pi(self):
        os.system("reboot")
        # add something here that adds to the restarts db that this restart was commanded

    def cease_comms(self):
        # I'm actually unsure of how to do this. Maybe do something with the GPIO pins so that the pi can't transmit
        self.parent.logger.critical("Ceasing all communications")
        # definitely should implement some sort of password and double verification to prevent accidental triggering
        raise NotImplementedError

    def set_system_clock(self, unix_epoch):
        # need to validate this works, and need to integrate updating RTC
        clk_id = time.CLOCK_REALTIME
        time.clock_settime(clk_id, float(unix_epoch))

    def reboot_gom(self):
        self.parent.gom.gom.reboot()

    def power_cycle(self, passcode):
        self.parent.gom.hard_reset(bool(passcode))

    def gom_outputs(self, output_channel, state, delay):
        self.parent.gom.set_output(output_channel, state, delay=delay)

    def gom_command(self, command_string: str, args: tuple):
        """Generalized Gom command - very powerful and possibly dangerous.
        Make sure you know exactly what you're doing when calling this."""

        method_to_call = getattr(self.parent.gom, command_string)
        try:
            result = method_to_call(*args)
            return result
        except TypeError:
            self.parent.logger.error(f"Incorrect args: {args} for gom method {command_string}")

    def general_command(self, method_name: str, args: tuple):
        """Generalized satellite action command - very powerful and possibly dangerous.
            Make sure you know exactly what you're doing when calling this."""
        method_to_call = getattr(self.parent, method_name)
        try:
            result = method_to_call(*args)
            return result
        except TypeError:
            self.parent.logger.error(f"Incorrect args: {args} for gom method {method_name}")
