from datetime import datetime
import utils.constants as constants
from utils.constants import FMEnum
import os
import time
from utils.log import get_log

logger = get_log()

class CommandDefinitions:
    def __init__(self, parent):
        self.parent = parent
        self.bootup_commands = {1: self.split}
        self.restart_commands = {}
        self.normal_commands = {
            1: self.run_opnav,
            2: self.change_attitude,
            3: self.electrolysis,
            5: self.set_parameter,
            6: self.gather_critical_telem,
            7: self.gather_basic_telem,
            8: self.gather_detailed_telem,
            9: self.verification,
            11: self.print_parameter,
            12: self.set_opnav_interval
        }

        self.electrolysis_commands = {
            1: self.electrolysis,
            5: self.set_parameter,
            6: self.gather_critical_telem,
            7: self.gather_basic_telem,
            8: self.gather_detailed_telem
        }

        self.low_battery_commands = {
            1: self.return_to_normal,
            2: self.set_exit_lowbatt_threshold,
            5: self.set_parameter,
            6: self.gather_critical_telem,
            7: self.gather_basic_telem,
            8: self.gather_detailed_telem,
        }

        self.safety_commands = {
            1: self.return_to_normal,
            # 2: Not Implemented/need clarification,
            5: self.set_parameter,
            6: self.gather_critical_telem,
            7: self.gather_basic_telem,
            8: self.gather_detailed_telem
        }

        self.opnav_commands = {
            1: self.run_opnav,
            2: self.set_opnav_interval
        }

        self.maneuver_commands = {
            1: self.run_opnav,
            2: self.change_attitude,
            9: self.burn}

        self.sensor_commands = {}

        self.test_commands = {
            2: self.split,
            3: self.run_opnav,
            6: self.gom_outputs}

        self.comms_commands = {}

        self.command_commands = {
            1: self.set_parameter,
            2: self.set_system_clock,
            3: self.reboot_pi,
            4: self.reboot_gom,
            5: self.power_cycle,
            6: self.gom_outputs,
            7: self.gom_command,
            8: self.general_command,
            170: self.cease_comms}

        self.COMMAND_DICT = {
            FMEnum.Boot.value: self.bootup_commands,
            FMEnum.Restart.value: self.restart_commands,
            FMEnum.Normal.value: self.normal_commands,
            FMEnum.LowBatterySafety.value: self.low_battery_commands,
            FMEnum.Safety.value: self.safety_commands,
            FMEnum.OpNav.value: self.opnav_commands,
            FMEnum.Maneuver.value: self.maneuver_commands,
            FMEnum.SensorMode.value: self.sensor_commands,
            FMEnum.TestMode.value: self.test_commands,
            FMEnum.CommsMode: self.comms_commands,
            FMEnum.Command.value: self.command_commands
        }

    def switch(self):
        pass

    def split(self):
        # for demo, delay of 0
        self.parent.gom.burnwire2(constants.SPLIT_BURNWIRE_DURATION, delay=0)
        # Tell gom to power burnwires in five seconds
        # self.parent.gom.burnwire2(constants.SPLIT_BURNWIRE_DURATION, delay=5)
        # start reading gyro info
        # read gyro rotation rate data after split - need to downlink these to make sure of successful split

    def run_opnav(self):
        logger.info("Running OpNav Pipeline")
        time.sleep(5)
        logger.info("OpNav calculations complete. Resulting attitude is [x, y, z, phi, theta]")
        # self.parent.run_opnav
        # raise NotImplementedError

    def set_parameter(self, **kwargs):
        """Changes the values of a variable in constants.py. Current implementation requires the 'name' kwarg to be a
        string which we can't pack/unpack """
        name = kwargs['name']
        value = kwargs['value']

        initial_value = getattr(constants, name)
        setattr(constants, name, value)
        changed_value = getattr(constants, name)
        self.parent.logger.info(f"Changed constant {name} from {initial_value} to {changed_value}")

        # TODO: implement "saving" and reading of parameters to a text file

    def set_exit_lowbatt_threshold(self, **kwargs):
        """Does the same thing as set_parameter, but only for the EXIT_LOW_BATTERY_MODE_THRESHOLD parameter. Only
        requires one kwarg and does some basic sanity checks on the passed value"""
        value = kwargs['value']
        try:
            assert 0 < value < 1.0 and float(value) is float
            if value >= constants.ENTER_LOW_BATTERY_MODE_THRESHOLD:
                self.parent.logger.error(
                    f"New value for Exit LB thresh must be less than current Enter LB thresh value")
                assert 0 == 1
            self.set_parameter(name="EXIT_LOW_BATTERY_MODE_THRESHOLD", value=value)
        except AssertionError:
            self.parent.logger.error(f"Incompatible value {value} for EXIT_LOW_BATTERY_MODE_THRESHOLD")

    def set_opnav_interval(self, **kwargs):
        """Does the same thing as set_parameter, but only for the OPNAV_INTERVAL parameter. Only
            requires one kwarg and does some basic sanity checks on the passed value. Value is in minutes"""
        value = kwargs['interval']
        try:
            assert value > 1
            self.set_parameter(name="OPNAV_INTERVAL", value=value)
        except AssertionError:
            self.parent.logger.error(f"Incompatible value {value} for SET_OPNAV_INTERVAL")

    def change_attitude(self, **kwargs):
        azimuth = kwargs['theta']
        elevation = kwargs['phi']

        # current_theta = telemetry.latest.theta
        # current_phi = telemetry.latest.phi

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

    def electrolysis(self, **kwargs):
        state = kwargs['state']
        delay = kwargs['delay']
        assert state is bool
        self.parent.gom.set_electrolysis(state, delay=delay)

    def burn(self, **kwargs):
        time_burn = kwargs['time']
        absolute = kwargs['absolute']

        if absolute:  # i.e. if we want to burn at a specific absolute time
            delay = time_burn - datetime.now()
        else:  # if we want to burn exactly x seconds from receiving the command
            delay = time

        if delay < 0:
            self.parent.logger.error("Burn delay calculated from time was negative. Aborting burn")
        else:
            self.parent.gom.glowplug(constants.GLOWPLUG_DURATION, delay=delay)

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

    def set_system_clock(self, **kwargs):  # Needs validation (talk to Viraj)
        # need to validate this works, and need to integrate updating RTC
        unix_epoch = kwargs['epoch']
        clk_id = time.CLOCK_REALTIME
        time.clock_settime(clk_id, float(unix_epoch))

    # TODO
    def print_parameter(self, **kwargs):
        index = kwargs["index"]
        value = constants
        self.parent.logger.info()

    def reboot_gom(self):
        self.parent.gom.gom.reboot()

    def power_cycle(self, **kwargs):
        passcode = kwargs['passcode']
        self.parent.gom.hard_reset(passcode)

    def gom_outputs(self, **kwargs):
        output_channel = kwargs['output_channel']
        state = kwargs.get('state', 0)  # if 'state' is not found in kwargs, assume we want it to turn off
        delay = kwargs.get('delay', 0)  # if 'delay' is not found in kwargs, assume we want it immediately
        self.parent.gom.set_output(output_channel, state, delay=delay)

    def gom_command(self, command_string: str, args: dict):
        """Generalized Gom command - very powerful and possibly dangerous.
        Make sure you know exactly what you're doing when calling this.
        Will not work for flight since we dont have any string packers/unpackers"""
        method_to_call = getattr(self.parent.gom, command_string)
        try:
            result = method_to_call(**args)
            return result
        except TypeError:
            self.parent.logger.error(f"Incorrect args: {args} for gom method {command_string}")

    def general_command(self, method_name: str, args: dict):
        """Generalized satellite action command - very powerful and possibly dangerous.
            Make sure you know exactly what you're doing when calling this.
            Will not work for flight since we dont have any string packers/unpackers"""

        method_to_call = getattr(self.parent, method_name)
        try:
            result = method_to_call(**args)
            return result
        except TypeError:
            self.parent.logger.error(f"Incorrect arguments: {args} for method {method_name}")
