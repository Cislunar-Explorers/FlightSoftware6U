from datetime import datetime

from utils.constants import FMEnum, NormalCommandEnum, SafetyCommandEnum, CommandCommandEnum
from utils.constants import LowBatterySafetyCommandEnum as LBSCEnum
import os
import time
from threading import Thread
from utils.constants import INTERVAL, STATE, DELAY, NAME, VALUE, a, b, M, team_identifier


class CommandDefinitions:
    def __init__(self, parent):
        self.parent = parent
        self.bootup_commands = {1: self.split}
        self.restart_commands = {}
        self.normal_commands = {
            # TODO: use CommandEnums instead of hardcoded values for all commands below
            NormalCommandEnum.RunOpNav.value: self.run_opnav,
            NormalCommandEnum.SetDesiredAttitude.value: self.change_attitude,
            NormalCommandEnum.SetElectrolysis.value: self.electrolysis,
            NormalCommandEnum.SetParam.value: self.set_parameter,
            NormalCommandEnum.CritTelem.value: self.gather_critical_telem,
            NormalCommandEnum.BasicTelem.value: self.gather_basic_telem,
            NormalCommandEnum.DetailedTelem.value: self.gather_detailed_telem,
            NormalCommandEnum.Verification.value: self.verification,
            NormalCommandEnum.GetParam.value: self.print_parameter,
            NormalCommandEnum.SetOpnavInterval.value: self.set_opnav_interval
        }

        self.low_battery_commands = {
            LBSCEnum.ExitLBSafetyMode.value: self.return_to_normal,
            LBSCEnum.SetExitLBSafetyMode.value: self.set_exit_lowbatt_threshold,
            LBSCEnum.SetParam.value: self.set_parameter,
            LBSCEnum.CritTelem.value: self.gather_critical_telem,
            LBSCEnum.BasicTelem.value: self.gather_basic_telem,
            LBSCEnum.DetailedTelem.value: self.gather_detailed_telem,
        }

        self.safety_commands = {
            SafetyCommandEnum.ExitSafetyMode.value: self.return_to_normal,
            # 2: Not Implemented/need clarification,
            SafetyCommandEnum.SetParameter.value: self.set_parameter,
            SafetyCommandEnum.CritTelem.value: self.gather_critical_telem,
            SafetyCommandEnum.BasicTelem.value: self.gather_basic_telem,
            SafetyCommandEnum.DetailedTelem.value: self.gather_detailed_telem
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
            5: self.separation_test,
            6: self.gom_outputs
        }

        self.comms_commands = {}

        self.command_commands = {
            CommandCommandEnum.SetParam.value: self.set_parameter,
            CommandCommandEnum.SetSystemTime.value: self.set_system_clock,
            CommandCommandEnum.RebootPi.value: self.reboot_pi,
            CommandCommandEnum.RebootGom.value: self.reboot_gom,
            CommandCommandEnum.PowerCycle.value: self.power_cycle,
            CommandCommandEnum.GomPin.value: self.gom_outputs,
            CommandCommandEnum.GomGeneralCmd.value: self.gom_command,
            CommandCommandEnum.GeneralCmd.value: self.general_command,
            CommandCommandEnum.CeaseComms.value: self.cease_comms
        }

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

        for value in self.COMMAND_DICT.values():
            value[0] = self.switch  # adds 0 to all of the dict entries in COMMAND_DICT

    def switch(self):
        self.parent.logger.critical("Manual FM switch commanded")

    def split(self):
        # for demo, delay of 0
        self.parent.gom.burnwire2(self.parent.constants.SPLIT_BURNWIRE_DURATION, delay=0)
        # Tell gom to power burnwires in five seconds
        # self.parent.gom.burnwire2(constants.SPLIT_BURNWIRE_DURATION, delay=5)
        # start reading gyro info
        # read gyro rotation rate data after split - need to downlink these to make sure of successful split

    def separation_test(self):
        gyro_threader = Thread(target=self.gyro_thread)
        gyro_threader.start()
        self.parent.gom.burnwire2(2)
        gyro_threader.join()

    def gyro_thread(self):
        freq = 250  # Hz
        duration = 4  # sec
        gyro_data = []
        self.parent.logger.info("Reading Gyro data (rad/s)")
        for i in range(int(duration * freq)):
            gyro_reading = self.parent.gyro.get_gyro()
            gyro_time = time.time()
            gyro_list = list(gyro_reading)
            gyro_list.append(gyro_time)
            gyro_data.append(gyro_list)
            time.sleep(1.0 / freq)

        # writes gyro data to gyro_data.txt. Caution, this file will be overwritten with every successive test
        self.parent.logger.info("Writing gyro data to file")
        with open('gyro_data.txt', 'w') as filehandle:
            filehandle.writelines("%s\n" % line for line in gyro_data)

    def run_opnav(self):
        self.parent.logger.info("Running OpNav Pipeline")
        time.sleep(10)
        self.parent.logger.info("OpNav calculations complete. Resulting attitude is [x, y, z, phi, theta]")
        if self.parent.flight_mode.flight_mode_id == 2:
            self.parent.flight_mode.last_opnav_run = datetime.now()
        # self.parent.run_opnav
        # raise NotImplementedError

    def set_parameter(self, **kwargs):
        """Changes the values of a variable in constants.py. Current implementation requires the 'name' kwarg to be a
        string which we can't pack/unpack """
        name = kwargs[NAME]
        value = kwargs[VALUE]

        initial_value = getattr(self.parent.constants, name)
        setattr(self.parent.constants, name, value)
        changed_value = getattr(self.parent.constants, name)
        self.parent.logger.info(f"Changed constant {name} from {initial_value} to {changed_value}")

        # TODO: implement "saving" and reading of parameters to a text file

    def set_exit_lowbatt_threshold(self, **kwargs):
        """Does the same thing as set_parameter, but only for the EXIT_LOW_BATTERY_MODE_THRESHOLD parameter. Only
        requires one kwarg and does some basic sanity checks on the passed value"""
        value = kwargs['value']
        try:
            assert 0 < value < 1.0 and float(value) is float
            if value >= self.parent.constants.ENTER_LOW_BATTERY_MODE_THRESHOLD:
                self.parent.logger.error(
                    f"New value for Exit LB thresh must be less than current Enter LB thresh value")
                assert False
            self.set_parameter(name="EXIT_LOW_BATTERY_MODE_THRESHOLD", value=value)
        except AssertionError:
            self.parent.logger.error(f"Incompatible value {value} for EXIT_LOW_BATTERY_MODE_THRESHOLD")

    def set_opnav_interval(self, **kwargs):
        """Does the same thing as set_parameter, but only for the OPNAV_INTERVAL parameter. Only
            requires one kwarg and does some basic sanity checks on the passed value. Value is in minutes"""
        value = kwargs[INTERVAL]
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
        data_block_sequence_num = 0
        team_bytes = team_identifier.to_bytes(4, 'big')
        num_blocks = 78
        operating_period_timestamp = 1609311600
        operating_period_base_seed = team_identifier ^ operating_period_timestamp

        data_transmission_sequence = bytes()

        for x in range(num_blocks):
            sequence_bytes = data_block_sequence_num.to_bytes(4, 'big')
            timestamp = time.time()
            seconds_bytes = int(timestamp).to_bytes(4, 'big')
            ms_bytes = int((timestamp - int(timestamp)) * (10 ** 6)).to_bytes(4, 'big')
            header = team_bytes + sequence_bytes + seconds_bytes + ms_bytes

            block_seed = operating_period_base_seed ^ data_block_sequence_num

            prn_length = 128 // 4
            prn = [None] * prn_length
            prn[0] = block_seed

            for i in range(1, prn_length):
                xn = (a * prn[i - 1] + b) % M
                prn[i] = xn

            data_field = bytes()
            for j in prn:
                data_field += j.to_bytes(4, 'big')

            data_block = header + data_field

            data_transmission_sequence += data_block
            data_block_sequence_num += 1

        return data_transmission_sequence

    def electrolysis(self, **kwargs):
        state = kwargs[STATE]
        delay = kwargs.get(DELAY, 0)
        assert type(state) is bool
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
            self.parent.gom.glowplug(self.parent.constants.GLOWPLUG_DURATION, delay=delay)

    def return_to_normal(self):
        self.parent.replace_flight_mode_by_id(FMEnum.Normal.value)

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

    def print_parameter(self, **kwargs):
        index = kwargs["index"]
        value = getattr(self.parent.constants, str(index))
        self.parent.logger.info(f"{index}:{value}")

    def reboot_gom(self):
        self.parent.gom.gom.reboot()

    def power_cycle(self, **kwargs):
        passcode = kwargs.get('passcode', 'bogus')
        self.parent.gom.hard_reset(passcode)

    def gom_outputs(self, **kwargs):
        output_channel = kwargs['output_channel']
        state = kwargs.get('state', 0)  # if 'state' is not found in kwargs, assume we want it to turn off
        delay = kwargs.get('delay', 0)  # if 'delay' is not found in kwargs, assume we want it immediately
        self.parent.gom.set_output(output_channel, state, delay=delay)

    def gom_command(self, command_string: str, args: dict):
        """Generalized Gom command - very powerful and possibly dangerous.
        Make sure you know exactly what you're doing when calling this."""
        method_to_call = getattr(self.parent.gom, command_string)
        try:
            result = method_to_call(**args)
            return result
        except TypeError:
            self.parent.logger.error(f"Incorrect args: {args} for gom method {command_string}")

    def general_command(self, method_name: str, args: dict):
        """Generalized satellite action command - very powerful and possibly dangerous.
            Make sure you know exactly what you're doing when calling this."""

        method_to_call = getattr(self.parent, method_name)
        try:
            result = method_to_call(**args)
            return result
        except TypeError:
            self.parent.logger.error(f"Incorrect arguments: {args} for method {method_name}")
