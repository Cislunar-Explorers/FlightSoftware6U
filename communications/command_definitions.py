from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from main import MainSatelliteThread
# for an explanation of the above 4 lines of code, see
# https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
# It lets your IDE know what type(self.parent) is, without causing any circular imports at runtime.

from utils.constants import LowBatterySafetyCommandEnum as LBSCEnum
import drivers.power.power_structs as ps
import time
from threading import Thread
from utils.constants import *
from json import load, dump
from utils.exceptions import CommandArgException

import os
import utils.parameters as params


def verification(**kwargs):
    """CQC Comms Verification
    For more info see https://cornell.app.box.com/file/766365097328
    Assuming a data rate of 50 bits/second, 30 minutes of data transmission gives 78 data blocks"""
    num_blocks = kwargs.get(NUM_BLOCKS)

    data_block_sequence_num = 0
    team_bytes = team_identifier.to_bytes(4, 'big')
    data_transmission_sequence = bytes()

    for x in range(num_blocks):
        # header calculation:
        sequence_bytes = data_block_sequence_num.to_bytes(4, 'big')
        # get current time
        timestamp = time.time()  # each block has its own timestamp
        # extract seconds and milliseconds from timestamp:
        seconds_int = int(timestamp)
        seconds_bytes = seconds_int.to_bytes(4, 'big')
        ms_bytes = int((timestamp - seconds_int) * (10 ** 6)).to_bytes(4, 'big')

        # concatenate header
        header = team_bytes + sequence_bytes + seconds_bytes + ms_bytes

        operating_period_base_seed = team_identifier ^ seconds_int  # team identifier xor with timestamp seconds
        block_seed = operating_period_base_seed ^ data_block_sequence_num  # xor previous with data block sequence num

        prn_length = 128 // 4  # integer division
        prn = [int()] * (prn_length + 1)  # preallocate memory for storing prn data
        prn[0] = block_seed  # x0 is the block seed

        for i in range(1, prn_length + 1):
            # algorithm defined in sec 4.4.2 of CommsProc rev 4
            xn = (a * prn[i - 1] + b) % M
            # if the mod operator above causes issues, anding with 32-bit 2**32 should do the trick
            prn[i] = xn

        prn.pop(0)  # get rid of the first value in the PRN, x0 is not included in PRN

        data_field = bytes()
        for j in prn:
            data_field += j.to_bytes(4, 'big')  # concatenate prn data into bytes

        data_block = header + data_field

        data_transmission_sequence += data_block  # concatenate data block into transmission sequence
        data_block_sequence_num += 1

    return data_transmission_sequence.hex()  # TODO instead of returning, add to comms queue


class CommandDefinitions:
    def __init__(self, parent: MainSatelliteThread):
        self.parent = parent
        self.bootup_commands = {1: self.split}
        self.restart_commands = {}
        self.normal_commands = {
            # TODO: use CommandEnums instead of hardcoded values for all commands below
            NormalCommandEnum.RunOpNav.value: self.run_opnav,
            # NormalCommandEnum.SetDesiredAttitude.value: self.change_attitude,
            NormalCommandEnum.SetElectrolysis.value: self.electrolysis,
            NormalCommandEnum.SetParam.value: self.set_parameter,
            NormalCommandEnum.CritTelem.value: self.gather_critical_telem,
            NormalCommandEnum.BasicTelem.value: self.gather_basic_telem,
            NormalCommandEnum.DetailedTelem.value: self.gather_detailed_telem,
            NormalCommandEnum.Verification.value: verification,
            NormalCommandEnum.GetParam.value: self.print_parameter,
            NormalCommandEnum.SetOpnavInterval.value: self.set_opnav_interval,
            NormalCommandEnum.ScheduleManeuver.value: self.schedule_maneuver,
            NormalCommandEnum.ACSPulsing.value: self.acs_pulse_timing,
            NormalCommandEnum.NemoWriteRegister.value: self.nemo_write_register,
            NormalCommandEnum.NemoReadRegister.value: self.nemo_read_register,
            NormalCommandEnum.NemoSetConfig.value: self.nemo_set_config,
            NormalCommandEnum.NemoPowerOff.value: self.nemo_power_off,
            NormalCommandEnum.NemoPowerOn.value: self.nemo_power_on,
            NormalCommandEnum.NemoReboot.value: self.nemo_reboot,
            NormalCommandEnum.NemoProcessRateData.value: self.nemo_process_rate_data,
            NormalCommandEnum.NemoProcessHistograms.value: self.nemo_process_histograms,
            NormalCommandEnum.GomConf1Set.value: self.set_gom_conf1,
            NormalCommandEnum.GomConf1Get.value: self.get_gom_conf1,
            NormalCommandEnum.GomConf2Set.value: self.set_gom_conf2,
            NormalCommandEnum.GomConf2Get.value: self.get_gom_conf2
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

        self.opnav_commands = {}
        self.maneuver_commands = {}
        self.sensor_commands = {}

        self.test_commands = {
            2: self.split,
            3: self.run_opnav,
            TestCommandEnum.ADCTest.value: self.adc_test,
            TestCommandEnum.SeparationTest.value: self.separation_test,
            6: self.gom_outputs,
            7: self.comms_driver_test,
            TestCommandEnum.PiShutdown.value: self.pi_shutdown,
            TestCommandEnum.RTCTest.value: self.rtc_test
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
        self.parent.gom.burnwire1(params.SPLIT_BURNWIRE_DURATION, delay=0)
        # Tell gom to power burnwires in five seconds
        # self.parent.gom.burnwire1(constants.SPLIT_BURNWIRE_DURATION, delay=5)
        # start reading gyro info
        # read gyro rotation rate data after split - need to downlink these to make sure of successful split

    def adc_test(self):
        # tests integration of ADC into the rest of the FSW
        self.parent.logger.info("Cold junction temperature for gyro sensor in Celsius:")
        self.parent.logger.info(self.parent.adc.get_gyro_temp())

        self.parent.logger.info(f"Pressure: {self.parent.adc.read_pressure()} psi")
        self.parent.logger.info(f"Temperature: {self.parent.adc.read_temperature()} deg C")

        self.parent.logger.info("Conversion sanity check: 25.6 degrees")
        self.parent.logger.info(self.parent.adc.convert_volt_to_temp(self.parent.adc.convert_temp_to_volt(25.6)))
        self.parent.logger.info("Conversion sanity check: 2.023 mV")
        self.parent.logger.info(self.parent.adc.convert_temp_to_volt(self.parent.adc.convert_volt_to_temp(2.023)))

    def rtc_test(self):
        self.parent.logger.info(f"Oscillator Disabled: {self.parent.rtc.ds3231.disable_oscillator}")
        self.parent.logger.info(f"RTC Temp: {self.parent.rtc.get_temp()}")
        self.parent.logger.info(f"RTC Time: {self.parent.rtc.get_time()}")
        # time.sleep(1)
        self.parent.logger.info(f"Setting RTC time to 1e9")
        self.parent.rtc.set_time(1e9)
        self.parent.logger.info(f"New RTC Time: {self.parent.rtc.get_time()}")
        # time.sleep(1)
        self.parent.logger.info(f"Incrementing RTC Time by 5555 seconds")
        self.parent.rtc.increment_rtc(5555)
        self.parent.logger.info(f"New RTC Time: {self.parent.rtc.get_time()}")
        self.parent.logger.info(f"Disabling Oscillator, waiting 10 seconds")
        self.parent.rtc.disable_oscillator()
        time.sleep(10)
        self.parent.logger.info(f"RTC Time after disabling oscillator: {self.parent.rtc.get_time()}")
        self.parent.logger.info(f"Enabling Oscillator, waiting 10 seconds")
        self.parent.rtc.enable_oscillator()
        time.sleep(10)
        self.parent.logger.info(f"RTC Time after re-enabling oscillator: {self.parent.rtc.get_time()}")
        self.parent.logger.info("Disabling Oscillator")
        self.parent.rtc.disable_oscillator()
        self.parent.handle_sigint()

    def separation_test(self):
        gyro_threader = Thread(target=self.gyro_thread)
        gyro_threader.start()
        self.parent.gom.burnwire1(2)
        gyro_threader.join()

    def gyro_thread(self):
        freq = 250  # Hz
        duration = 3  # sec
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
        """Schedules Opnav mode into the FM queue"""
        self.parent.FMQueue.put(FMEnum.OpNav.value)

    def set_parameter(self, **kwargs):
        """Changes the values of a variable in constants.py. Current implementation requires the 'name' kwarg to be a
        string which we can't pack/unpack """
        name = kwargs[NAME]
        value = kwargs[VALUE]
        hard_set = kwargs[HARD_SET]
        initial_value = getattr(params, name)
        params.__setattr__(name, value)

        # Hard sets new parameter value into JSON file
        if hard_set:
            with open(PARAMETERS_JSON_PATH) as f:
                json_parameter_dict = load(f)
            json_parameter_dict[name] = value
            dump(json_parameter_dict, open(PARAMETERS_JSON_PATH, 'w'), indent=0)

        acknowledgement = self.parent.downlink_handler.pack_downlink(
            self.parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.SetParam.value, successful=True)
        self.parent.downlink_queue.put(acknowledgement)

        self.parent.logger.info(f"Changed constant {name} from {initial_value} to {value}")

    def set_exit_lowbatt_threshold(self, **kwargs):
        """Does the same thing as set_parameter, but only for the EXIT_LOW_BATTERY_MODE_THRESHOLD parameter. Only
        requires one kwarg and does some basic sanity checks on the passed value"""
        value = kwargs['value']
        try:
            assert 0 < value < 1.0 and float(value) is float
            if value >= params.ENTER_LOW_BATTERY_MODE_THRESHOLD:
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

    # def change_attitude(self, **kwargs):
    #     theta = kwargs.get(AZIMUTH)
    #     phi = kwargs.get(ELEVATION)  # angle down from z-axis of ECI frame
    #
    #     assert 0 <= theta < 6.28318530718
    #     assert 0 <= phi < 3.14159265359
    #
    #     self.parent.reorientation_queue.put((theta, phi))

    def acs_pulse_timing(self, **kwargs):
        pulse_start_time = kwargs[START]  # float, seconds
        pulse_duration = kwargs[PULSE_DURATION]  # ushort, milliseconds
        pulse_num = kwargs[PULSE_NUM]  # ushort, number
        pulse_dt = kwargs[PULSE_DT]  # ushort, milliseconds

        try:
            assert pulse_start_time > time.time()
            assert pulse_duration > 0
            assert pulse_num >= 0
            assert pulse_dt >= 0
        except AssertionError:
            raise CommandArgException

        self.parent.reorientation_queue.put((pulse_start_time, pulse_duration, pulse_num, pulse_dt))

    def gather_critical_telem(self):
        # here we want to only gather the most critical telemetry values so that we spend the least electricity
        # downlinking them (think about a low-power scenario where the most important thing is our power in and out)
        raise NotImplementedError

    def gather_basic_telem(self):
        # what's defined in section 3.6.1 of https://cornell.app.box.com/file/629596158344 would be a good packet
        return self.parent.telemetry.standard_packet_dict()

    def gather_detailed_telem(self):
        # here we'd gather as much data about the satellite as possible
        raise NotImplementedError

    def electrolysis(self, **kwargs):
        state = kwargs[STATE]
        delay = kwargs.get(DELAY, 0)
        assert type(state) is bool
        self.parent.gom.set_electrolysis(state, delay=delay)

    # def burn(self, **kwargs):
    #     time_burn = kwargs['time']
    #     absolute = kwargs['absolute']
    #
    #     if absolute:  # i.e. if we want to burn at a specific absolute time
    #         delay = time_burn - datetime.now()
    #     else:  # if we want to burn exactly x seconds from receiving the command
    #         delay = time
    #
    #     if delay < 0:
    #         self.parent.logger.error("Burn delay calculated from time was negative. Aborting burn")
    #     else:
    #         self.parent.gom.glowplug(params.GLOWPLUG_DURATION, delay=delay)

    def schedule_maneuver(self, **kwargs):
        time_burn = kwargs['time']
        self.parent.logger.info("Scheduling a maneuver at: " + str(float(time_burn)))
        self.set_parameter(name="SCHEDULED_BURN_TIME", value=float(time_burn), hard_set=True)
        self.parent.maneuver_queue.put(FMEnum.Maneuver.value)

    def return_to_normal(self):
        self.parent.replace_flight_mode_by_id(FMEnum.Normal.value)

    @staticmethod
    def reboot_pi():
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
        value = getattr(params, str(index))
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

    def comms_driver_test(self):

        gyro = self.parent.gyro.get_gyro()

        fx_data = self.parent.downlink_handler.pack_downlink(FMEnum.TestMode.value,
                                                             TestCommandEnum.CommsDriver.value,
                                                             gyro1=gyro[0], gyro2=gyro[1], gyro3=gyro[2])

        time.sleep(5)
        self.parent.radio.transmit(fx_data)

    def pi_shutdown(self):
        os.system('sudo poweroff')

    def edit_file_at_line(self, **kwargs):

        file_path = kwargs['file_path']
        line_number = kwargs['line_number']
        new_line = kwargs['new_line']

        # Open and copy file
        original_file = open(file_path, 'r+')
        original_file_lines = original_file.readlines()
        new_file_lines = original_file_lines[:]

        # Modify copy at designated line
        new_file_lines[line_number] = new_line + ' \n'

        # Write copy onto original file and original file into a backup
        backup_file = open('backup_' + file_path, 'w')
        backup_file.writelines(original_file_lines)
        original_file.writelines(new_file_lines)

    def insert_line_in_file(self, **kwargs):

        file_path = kwargs['file_path']
        line_number = kwargs['line_number']
        new_line = kwargs['new_line']

        # Get original file contents
        original_file = open(file_path, 'r+')
        my_file_lines = original_file.readlines()
        pre_contents = my_file_lines[:line_number]
        post_contents = my_file_lines[line_number:]

        # Write new line into file
        original_file.seek(0)
        original_file.writelines(pre_contents + [new_line + ' \n'] + post_contents)

    def nemo_write_register(self, **kwargs):
        if self.parent.nemo_manager is not None:
            reg_address = kwargs[REG_ADDRESS]
            values = [kwargs[REG_VALUE]]

            self.parent.nemo_manager.write_register(reg_address, values)
        else:
            self.parent.logger.error("CMD: nemo_write_register() failed, nemo_manager not initialized")

    def nemo_read_register(self, **kwargs):
        if self.parent.nemo_manager is not None:
            reg_address = kwargs[REG_ADDRESS]
            size = kwargs[REG_SIZE]

            self.parent.nemo_manager.read_register(reg_address, size)
        else:
            self.parent.logger.error("CMD: nemo_read_register() failed, nemo_manager not initialized")

    def nemo_set_config(self, **kwargs):
        if self.parent.nemo_manager is not None:
            self.parent.nemo_manager.set_config(**kwargs)
        else:
            self.parent.logger.error("CMD: nemo_set_config() failed, nemo_manager not initialized")

    def nemo_power_off(self):
        if self.parent.nemo_manager is not None:
            self.parent.nemo_manager.power_off()
        else:
            self.parent.logger.error("CMD: nemo_power_off() failed, nemo_manager not initialized")

    def nemo_power_on(self):
        if self.parent.nemo_manager is not None:
            self.parent.nemo_manager.power_on()
        else:
            self.parent.logger.error("CMD: nemo_power_on() failed, nemo_manager not initialized")

    def nemo_reboot(self):
        if self.parent.nemo_manager is not None:
            self.parent.nemo_manager.reboot()
        else:
            self.parent.logger.error("CMD: nemo_reboot() failed, nemo_manager not initialized")

    def nemo_process_rate_data(self, **kwargs):
        if self.parent.nemo_manager is not None:
            t_start = kwargs[T_START]
            t_stop = kwargs[T_STOP]
            decimation_factor = kwargs[DECIMATION_FACTOR]

            self.parent.nemo_manager.process_rate_data(t_start, t_stop, decimation_factor)
        else:
            self.parent.logger.error("CMD: nemo_process_rate_data() failed, nemo_manager not initialized")

    def nemo_process_histograms(self, **kwargs):
        if self.parent.nemo_manager is not None:
            t_start = kwargs[T_START]
            t_stop = kwargs[T_STOP]
            decimation_factor = kwargs[DECIMATION_FACTOR]

            self.parent.nemo_manager.process_histograms(t_start, t_stop, decimation_factor)
        else:
            self.parent.logger.error("CMD: nemo_process_histograms() failed, nemo_manager not initialized")

    def set_gom_conf1(self, **kwargs):
        new_config = eps_config_from_dict(**kwargs)
        self.parent.logger.info("New config to be set:")
        ps.displayConfig(new_config)

        if self.parent.gom is not None:
            try:
                self.parent.gom.pc.config_set(new_config)
                updated_config: ps.eps_config_t = self.parent.gom.pc.config_get()

                new_config_dict = dict_from_eps_config(updated_config)
                acknowledgement = self.parent.downlink_handler.pack_downlink(
                    self.parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.GomConf1Set.value,
                    **new_config_dict)

            except Exception:
                self.parent.logger.error("Could not set new gom config")
                acknowledgement = self.parent.downlink_handler.pack_downlink(
                    self.parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.CommandStatus.value,
                    fmid=self.parent.flight_mode.flight_mode_id, cid=NormalCommandEnum.GomConf1Set.value,
                    successful=False)

            self.parent.downlink_queue.put(acknowledgement)

    def get_gom_conf1(self, **kwargs):
        if self.parent.gom is not None:
            current_config = self.parent.gom.get_health_data(level="config")
            ps.displayConfig(current_config)
            current_config_dict = dict_from_eps_config(current_config)
            acknowledgement = self.parent.downlink_handler.pack_downlink(
                self.parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.GomConf1Get.value,
                **current_config_dict)
            self.parent.downlink_queue.put(acknowledgement)

    def set_gom_conf2(self, **kwargs):
        if self.parent.gom is not None:
            new_conf2 = eps_config2_from_dict(kwargs)
            self.parent.gom.pc.config2_set(new_conf2)
            self.parent.gom.pc.config2_cmd(2)

    def get_gom_conf2(self, **kwargs):
        if self.parent.gom is not None:
            current_conf2 = self.parent.gom.get_health_data(level='config2')
            ps.displayConfig2(current_conf2)
            current_config2_dict = dict_from_eps_config2(current_conf2)
            acknowledgement = self.parent.downlink_handler.pack_downlink(
                self.parent.downlink_counter, FMEnum.Normal.value, NormalCommandEnum.GomConf2Get.value,
                **current_config2_dict)
            self.parent.downlink_queue.put(acknowledgement)


def dict_from_eps_config(config: ps.eps_config_t) -> dict:
    return {PPT_MODE: config.ppt_mode,
            BATTHEATERMODE: bool(config.battheater_mode),
            BATTHEATERLOW: config.battheater_low,
            BATTHEATERHIGH: config.battheater_high,
            OUTPUT_NORMAL1: bool(config.output_normal_value[0]),
            OUTPUT_NORMAL2: bool(config.output_normal_value[1]),
            OUTPUT_NORMAL3: bool(config.output_normal_value[2]),
            OUTPUT_NORMAL4: bool(config.output_normal_value[3]),
            OUTPUT_NORMAL5: bool(config.output_normal_value[4]),
            OUTPUT_NORMAL6: bool(config.output_normal_value[5]),
            OUTPUT_NORMAL7: bool(config.output_normal_value[6]),
            OUTPUT_NORMAL8: bool(config.output_normal_value[7]),
            OUTPUT_SAFE1: bool(config.output_safe_value[0]),
            OUTPUT_SAFE2: bool(config.output_safe_value[1]),
            OUTPUT_SAFE3: bool(config.output_safe_value[2]),
            OUTPUT_SAFE4: bool(config.output_safe_value[3]),
            OUTPUT_SAFE5: bool(config.output_safe_value[4]),
            OUTPUT_SAFE6: bool(config.output_safe_value[5]),
            OUTPUT_SAFE7: bool(config.output_safe_value[6]),
            OUTPUT_SAFE8: bool(config.output_safe_value[7]),
            OUTPUT_ON_DELAY: config.output_initial_on_delay[0],
            OUTPUT_OFF_DELAY: config.output_initial_off_delay[0],
            VBOOST1: config.vboost[0],
            VBOOST2: config.vboost[1],
            VBOOST3: config.vboost[2],
            }


def eps_config_from_dict(**kwargs) -> ps.eps_config_t:
    ppt_mode = kwargs.get(PPT_MODE)
    heater_mode = int(kwargs.get(BATTHEATERMODE))  # BATTHEATERMODE is transmitted as a bool, then cast to 0/1
    heater_low = kwargs.get(BATTHEATERLOW)
    heater_high = kwargs.get(BATTHEATERHIGH)
    normal_output = [kwargs.get(OUTPUT_NORMAL1),
                     kwargs.get(OUTPUT_NORMAL2),
                     kwargs.get(OUTPUT_NORMAL3),
                     kwargs.get(OUTPUT_NORMAL4),
                     kwargs.get(OUTPUT_NORMAL5),
                     kwargs.get(OUTPUT_NORMAL6),
                     kwargs.get(OUTPUT_NORMAL7),
                     kwargs.get(OUTPUT_NORMAL8)]

    normal_output = list(map(int, normal_output))  # transmitted as bools, convert to ints

    safe_output = [kwargs.get(OUTPUT_SAFE1),
                   kwargs.get(OUTPUT_SAFE2),
                   kwargs.get(OUTPUT_SAFE3),
                   kwargs.get(OUTPUT_SAFE4),
                   kwargs.get(OUTPUT_SAFE5),
                   kwargs.get(OUTPUT_SAFE6),
                   kwargs.get(OUTPUT_SAFE7),
                   kwargs.get(OUTPUT_SAFE8)]

    safe_output = list(map(int, safe_output))  # transmitted as bools, convert to ints

    # this means that all outputs have the same on/off delay
    initial_on_delay = [kwargs.get(OUTPUT_ON_DELAY)] * 8
    initial_off_delay = [kwargs.get(OUTPUT_OFF_DELAY)] * 8

    vboost = [kwargs.get(VBOOST1), kwargs.get(VBOOST2), kwargs.get(VBOOST3)]

    new_config = ps.eps_config_t()
    new_config.ppt_mode = ppt_mode
    new_config.battheater_mode = heater_mode
    new_config.battheater_low = heater_low
    new_config.battheater_high = heater_high

    new_config.output_normal_value[0] = normal_output[0]
    new_config.output_normal_value[1] = normal_output[1]
    new_config.output_normal_value[2] = normal_output[2]
    new_config.output_normal_value[3] = normal_output[3]
    new_config.output_normal_value[4] = normal_output[4]
    new_config.output_normal_value[5] = normal_output[5]
    new_config.output_normal_value[6] = normal_output[6]
    new_config.output_normal_value[7] = normal_output[7]

    new_config.output_safe_value[0] = safe_output[0]
    new_config.output_safe_value[1] = safe_output[1]
    new_config.output_safe_value[2] = safe_output[2]
    new_config.output_safe_value[3] = safe_output[3]
    new_config.output_safe_value[4] = safe_output[4]
    new_config.output_safe_value[5] = safe_output[5]
    new_config.output_safe_value[6] = safe_output[6]
    new_config.output_safe_value[7] = safe_output[7]

    new_config.output_initial_on_delay[0] = initial_on_delay[0]
    new_config.output_initial_on_delay[1] = initial_on_delay[1]
    new_config.output_initial_on_delay[2] = initial_on_delay[2]
    new_config.output_initial_on_delay[3] = initial_on_delay[3]
    new_config.output_initial_on_delay[4] = initial_on_delay[4]
    new_config.output_initial_on_delay[5] = initial_on_delay[5]
    new_config.output_initial_on_delay[6] = initial_on_delay[6]
    new_config.output_initial_on_delay[7] = initial_on_delay[7]

    new_config.output_initial_off_delay[0] = initial_off_delay[0]
    new_config.output_initial_off_delay[1] = initial_off_delay[1]
    new_config.output_initial_off_delay[2] = initial_off_delay[2]
    new_config.output_initial_off_delay[3] = initial_off_delay[3]
    new_config.output_initial_off_delay[4] = initial_off_delay[4]
    new_config.output_initial_off_delay[5] = initial_off_delay[5]
    new_config.output_initial_off_delay[6] = initial_off_delay[6]
    new_config.output_initial_off_delay[7] = initial_off_delay[7]

    new_config.vboost[0] = vboost[0]
    new_config.vboost[1] = vboost[1]
    new_config.vboost[2] = vboost[2]

    return new_config


def eps_config2_from_dict(config_dict: dict) -> ps.eps_config2_t:
    gom_conf2 = ps.eps_config2_t()

    max_voltage = config_dict.get(MAX_VOLTAGE)
    normal_voltage = config_dict.get(NORM_VOLTAGE)
    safe_voltage = config_dict.get(SAFE_VOLTAGE)
    crit_voltage = config_dict.get(CRIT_VOLTAGE)

    gom_conf2.batt_maxvoltage = max_voltage
    gom_conf2.batt_normalvoltage = normal_voltage
    gom_conf2.batt_safevoltage = safe_voltage
    gom_conf2.criticalvoltage = crit_voltage

    return gom_conf2


def dict_from_eps_config2(conf2: ps.eps_config2_t) -> dict:
    return {MAX_VOLTAGE: conf2.batt_maxvoltage,
            NORM_VOLTAGE: conf2.batt_normalvoltage,
            SAFE_VOLTAGE: conf2.batt_safevoltage,
            CRIT_VOLTAGE: conf2.batt_criticalvoltage}
