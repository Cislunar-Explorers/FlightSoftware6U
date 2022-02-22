from __future__ import annotations
import logging
from typing import TYPE_CHECKING, Dict, List, Optional, Union, cast
from communications.codec import Codec
from communications.commands import Command
from communications import codecs
from utils import parameter_utils

from utils import gom_util

if TYPE_CHECKING:
    from main import MainSatelliteThread
# for an explanation of the above 4 lines of code, see
# https://stackoverflow.com/questions/39740632/python-type-hinting-without-cyclic-imports
# It lets your IDE know what type(parent) is, without causing any circular imports at runtime.

import drivers.power.power_structs as ps
import time
import hashlib
from utils.constants import (
    FMEnum,
    CommandEnum,
    CommandKwargs as ck,
    DownlinkKwargs as dk,
    GomConfKwargs,
)
import utils.constants as consts
from utils.exceptions import CommandArgException
import subprocess


import os
import utils.parameters as params


class FM_Switch(Command):
    """Base class for doing manual FM switches on command"""

    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        logging.critical(f"Manual FM change commanded: {self.id}")
        parent.replace_flight_mode_by_id(self.id)


class BootSwitch(FM_Switch):
    id = CommandEnum.Boot


class RestartSwitch(FM_Switch):
    id = CommandEnum.Restart


class NormalSwitch(FM_Switch):
    id = CommandEnum.Normal


class SafetySwitch(FM_Switch):
    id = CommandEnum.Safety


class LowBatterySwitch(FM_Switch):
    id = CommandEnum.LowBatterySafety


class OpnavSwitch(FM_Switch):
    id = CommandEnum.OpNav


class ManeuverSwitch(FM_Switch):
    id = CommandEnum.Maneuver


class SensorSwitch(FM_Switch):
    id = CommandEnum.SensorMode


class TestSwitch(FM_Switch):
    id = CommandEnum.TestMode


class CommsSwitch(FM_Switch):
    id = CommandEnum.CommsMode


class CommandSwitch(FM_Switch):
    id = CommandEnum.CommandMode


class AttitudeAdjustmentSwitch(FM_Switch):
    id = CommandEnum.AttitudeAdjustment


class separation_test(Command):
    id = CommandEnum.SeparationTest
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        parent.gom.burnwire.pulse(consts.SPLIT_BURNWIRE_DURATION, asynchronous=True)
        gyro_data = []
        logging.info("Reading Gyro data (rad/s)")
        for _ in range(1000):
            gyro_reading = parent.gyro.get_gyro()
            gyro_time = time.time()
            gyro_list = list(gyro_reading)
            gyro_list.append(gyro_time)
            gyro_data.append(gyro_list)

        # writes gyro data to gyro_data.txt. Caution, this file will be overwritten with every successive test
        logging.info("Writing gyro data to file")
        with open("gyro_data.txt", "w") as filehandle:
            filehandle.writelines("%s\n" % line for line in gyro_data)


class run_opnav(Command):
    id = CommandEnum.ScheduleOpnav
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        """Schedules Opnav mode into the FM queue"""
        parent.FMQueue.put(FMEnum.OpNav)


class exit_low_batt_thresh(Command):
    """Does the same thing as set_parameter, but only for the EXIT_LOW_BATTERY_MODE_THRESHOLD parameter. Only
    requires one kwarg and does some basic sanity checks on the passed value"""

    id = CommandEnum.LowBattThresh
    uplink_codecs = [Codec(ck.VBATT, "ushort")]
    downlink_codecs = []

    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        value = kwargs[ck.VBATT]
        try:
            assert 6000 < value < 8400 and float(value) is float
            parameter_utils.set_parameter(
                "EXIT_LOW_BATTERY_MODE_THRESHOLD", value, False
            )
        except AssertionError:
            logging.error(
                f"Incompatible value {value} for EXIT_LOW_BATTERY_MODE_THRESHOLD"
            )


class set_opnav_interval(Command):
    id = CommandEnum.SetOpnavInterval
    uplink_codecs = [Codec(ck.INTERVAL, "ushort")]
    downlink_codecs = []

    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        """Does the same thing as set_parameter, but only for the OPNAV_INTERVAL parameter. Only
            requires one kwarg and does some basic sanity checks on the passed value. Value is in minutes"""
        value = kwargs[ck.INTERVAL]
        try:
            assert 1000 > value > 10
            parameter_utils.set_parameter("OPNAV_INTERVAL", value, hard_set=False)
        except AssertionError:
            logging.error(f"Incompatible value {value} for SET_OPNAV_INTERVAL")


# Future command:
# def change_attitude(self, **kwargs):
#     theta = kwargs.get(AZIMUTH)
#     phi = kwargs.get(ELEVATION)  # angle down from z-axis of ECI frame
#
#     assert 0 <= theta < 6.28318530718
#     assert 0 <= phi < 3.14159265359
#
#     parent.reorientation_queue.put((theta, phi))


class acs_pulse_timing(Command):
    id = CommandEnum.ACSPulsing
    uplink_codecs = [
        Codec(ck.START, "double"),
        Codec(ck.PULSE_DURATION, "ushort"),
        Codec(ck.PULSE_NUM, "ushort"),
        Codec(ck.PULSE_DT, "ushort"),
    ]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        pulse_start_time = kwargs[ck.START]  # double, seconds
        pulse_duration = kwargs[ck.PULSE_DURATION]  # ushort, milliseconds
        pulse_num = kwargs[ck.PULSE_NUM]  # ushort, number
        pulse_dt = kwargs[ck.PULSE_DT]  # ushort, milliseconds

        try:
            assert pulse_start_time > time.time()
            assert pulse_duration > 0
            assert pulse_num >= 0 and pulse_num < 50
            assert pulse_dt >= 0
        except AssertionError:
            raise CommandArgException

        parent.reorientation_queue.put(
            (pulse_start_time, pulse_duration, pulse_num, pulse_dt)
        )


class critical_telem(Command):
    id = CommandEnum.CritTelem
    uplink_codecs = []
    downlink_codecs = [
        codecs.VBATT_codec,
        codecs.CURSUN_codec,
        codecs.CURSYS_codec,
        codecs.GOM_BATTMODE_codec,
        codecs.GOM_PPT_MODE_codec,
    ]

    def _method(self, parent: MainSatelliteThread, **kwargs):
        # here we want to only gather the most critical telemetry values so that we spend the least electricity
        # downlinking them (think about a low-power scenario where the most important thing is our power in and out)
        return parent.telemetry.minimal_packet()


class basic_telem(Command):
    id = CommandEnum.BasicTelem
    uplink_codecs = []
    downlink_codecs = [
        codecs.RTC_TIME_codec,
        codecs.POS_X_codec,
        codecs.POS_Y_codec,
        codecs.POS_Z_codec,
        codecs.ATT_1_codec,
        codecs.ATT_2_codec,
        codecs.ATT_3_codec,
        codecs.ATT_4_codec,
        codecs.HK_TEMP_1_codec,
        codecs.HK_TEMP_2_codec,
        codecs.HK_TEMP_3_codec,
        codecs.HK_TEMP_4_codec,
        codecs.GYRO_TEMP_codec,
        codecs.THERMOCOUPLE_TEMP_codec,
        codecs.CURIN1_codec,
        codecs.CURIN2_codec,
        codecs.CURIN3_codec,
        codecs.VBOOST_1_codec,
        codecs.VBOOST_2_codec,
        codecs.VBOOST_3_codec,
        codecs.CURSYS_codec,
        codecs.VBATT_codec,
        codecs.PROP_TANK_PRESSURE_codec,
    ]

    def _method(
        self, parent: MainSatelliteThread, **kwargs
    ) -> Dict[str, Union[float, int]]:
        # what's defined in section 3.6.1 of https://cornell.app.box.com/file/629596158344 would be a good packet
        return parent.telemetry.standard_packet_dict()


class detailed_telem(Command):
    id = CommandEnum.DetailedTelem
    uplink_codecs = []
    downlink_codecs = [
        codecs.TIME_codec,
        codecs.VBOOST_1_codec,
        codecs.VBOOST_2_codec,
        codecs.VBOOST_3_codec,
        codecs.VBATT_codec,
        codecs.CURIN1_codec,
        codecs.CURIN2_codec,
        codecs.CURIN3_codec,
        codecs.CURSUN_codec,
        codecs.CURSYS_codec,
        codecs.RESERVED1_codec,
        codecs.CUROUT1_codec,
        codecs.CUROUT2_codec,
        codecs.CUROUT3_codec,
        codecs.CUROUT4_codec,
        codecs.CUROUT5_codec,
        codecs.CUROUT6_codec,
        codecs.OUTPUTS_codec,
        codecs.LATCHUPS1_codec,
        codecs.LATCHUPS2_codec,
        codecs.LATCHUPS3_codec,
        codecs.LATCHUPS4_codec,
        codecs.LATCHUPS5_codec,
        codecs.LATCHUPS6_codec,
        codecs.WDT_TIME_LEFT_I2C_codec,
        codecs.WDT_TIME_LEFT_GND_codec,
        codecs.WDT_COUNTS_I2C_codec,
        codecs.WDT_COUNTS_GND_codec,
        codecs.GOM_BOOTS_codec,
        codecs.GOM_BOOTCAUSE_codec,
        codecs.GOM_BATTMODE_codec,
        codecs.HK_TEMP_1_codec,
        codecs.HK_TEMP_2_codec,
        codecs.HK_TEMP_3_codec,
        codecs.HK_TEMP_4_codec,
        codecs.GOM_PPT_MODE_codec,
        codecs.RESERVED2_codec,
        codecs.RTC_TIME_codec,
        codecs.RPI_CPU_codec,
        codecs.RPI_RAM_codec,
        codecs.RPI_DSK_codec,
        codecs.RPI_TEMP_codec,
        codecs.RPI_BOOT_codec,
        codecs.RPI_UPTIME_codec,
        codecs.GYROX_codec,
        codecs.GYROY_codec,
        codecs.GYROZ_codec,
        codecs.ACCX_codec,
        codecs.ACCY_codec,
        codecs.ACCZ_codec,
        codecs.MAGX_codec,
        codecs.MAGY_codec,
        codecs.MAGZ_codec,
        codecs.GYRO_TEMP_codec,
        codecs.THERMOCOUPLE_TEMP_codec,
        codecs.PROP_TANK_PRESSURE_codec,
        codecs.POS_X_codec,
        codecs.POS_Y_codec,
        codecs.POS_Z_codec,
        codecs.ATT_1_codec,
        codecs.ATT_2_codec,
        codecs.ATT_3_codec,
        codecs.ATT_4_codec,
    ]
    # Needs validation

    def _method(self, parent: MainSatelliteThread, **kwargs) -> Dict[dk, int | float]:
        return parent.telemetry.detailed_packet_dict()


class electrolysis(Command):
    id = CommandEnum.SetElectrolysis
    uplink_codecs = [Codec(ck.STATE, "bool")]
    downlink_codecs = []

    # Needs validation
    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        state = kwargs[ck.STATE]
        parameter_utils.set_parameter(
            "WANT_TO_ELECTROLYZE", bool(state), hard_set=False
        )


class ignore_low_batt(Command):
    """This is obviously a very dangerous command. It's mainly meant for testing on the ground"""

    id = CommandEnum.IgnoreLowBatt
    uplink_codecs = [Codec(ck.IGNORE, "bool")]
    downlink_codecs = []

    # Needs validation
    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        ignore = kwargs[ck.IGNORE]
        parameter_utils.set_parameter(
            "IGNORE_LOW_BATTERY", bool(ignore), hard_set=False
        )


class schedule_maneuver(Command):
    id = CommandEnum.ScheduleManeuver
    uplink_codecs = [Codec(ck.MANEUVER_TIME, "double")]
    downlink_codecs = []

    # Needs validation
    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        time_burn = kwargs[ck.MANEUVER_TIME]
        logging.info("Scheduling a maneuver at: " + str(float(time_burn)))
        if params.SCHEDULED_BURN_TIME > 0:
            parent.maneuver_queue.put(params.SCHEDULED_BURN_TIME)
        parent.maneuver_queue.put(float(time_burn))

        smallest_time_burn = parent.maneuver_queue.get()
        parameter_utils.set_parameter(
            name="SCHEDULED_BURN_TIME",
            value=smallest_time_burn,
            hard_set=consts.FOR_FLIGHT,
        )


class reboot(Command):
    id = CommandEnum.RebootPi
    uplink_codecs = []
    downlink_codecs = []

    # Needs validation
    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        # TODO: make rebooting less janky
        os.system("sudo reboot")
        # add something here that adds to the restarts db that this restart was commanded


class cease_comms(Command):
    """This is an FCC requirement.
    We need to be able to command our spacecraft to stop transmitting if we become a nuisance to other radio stuff"""

    id = CommandEnum.CeaseComms
    uplink_codecs = [Codec(ck.PASSWORD, "string")]
    downlink_codecs = []

    # Needs validation
    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        pword = kwargs[ck.PASSWORD]
        if pword == 123:  # TODO, change
            logging.critical("Ceasing all communications")
            # definitely should implement some sort of verification to prevent accidental triggering
            raise NotImplementedError


class set_system_time(Command):
    id = CommandEnum.SetSystemTime
    uplink_codecs = [Codec(ck.SYS_TIME, "double")]
    downlink_codecs = [Codec(dk.TIME, "double")]

    # Needs validation
    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Dict[str, float]:
        # need to validate this works, and need to integrate updating RTC
        unix_epoch = kwargs[ck.SYS_TIME]
        clk_id = time.CLOCK_REALTIME
        time.clock_settime(clk_id, float(unix_epoch))
        return {dk.TIME: time.time()}


class get_param(Command):
    """Gets the value of an int or float parameter, logs it, and downlinks it as a (double) float"""

    id = CommandEnum.GetParam
    uplink_codecs = [Codec(ck.NAME, "string")]
    downlink_codecs = [Codec(dk.VALUE, "double")]

    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Dict[str, float]:
        name = kwargs[ck.NAME]
        value = getattr(params, name)
        logging.info(f"{name}: {value}")
        return {dk.VALUE: value}


class reboot_gom(Command):
    id = CommandEnum.RebootGom
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        parent.gom.driver.reboot()


class power_cycle(Command):
    """Powers off the entire spacecraft for 400ms"""

    id = CommandEnum.PowerCycle
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        # TODO: safe shutdown RPi before hard reset
        parent.gom.hard_reset(True)


class gom_outputs(Command):
    id = CommandEnum.GomPin
    uplink_codecs = [
        Codec(ck.OUTPUT_CHANNEL, "uint8"),
        Codec(ck.STATE, "bool"),
        Codec(ck.DELAY, "ushort"),
    ]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        output_channel = kwargs[ck.OUTPUT_CHANNEL]
        # if 'state' is not found in kwargs, assume we want it to turn off
        state = kwargs.get(ck.STATE, 0)
        # if 'delay' is not found in kwargs, assume we want it immediately
        delay = kwargs.get(ck.DELAY, 0)

        parent.gom.set_single_output(output_channel, int(state), delay=delay)


class pi_shutdown(Command):
    id = CommandEnum.PiShutdown
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        # TODO: do this more gracefully
        os.system("sudo poweroff")


class set_file_to_update(Command):
    """Downlink checksum of file blocks and any missing block numbers"""

    id = CommandEnum.SetUpdatePath
    uplink_codecs = [Codec(ck.FILE_PATH, "long_string")]
    downlink_codecs = []

    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        file_path = kwargs[ck.FILE_PATH]
        parameter_utils.set_parameter("FILE_UPDATE_PATH", file_path, False)


class add_file_block(Command):
    id = CommandEnum.AddFileBlock
    uplink_codecs = [
        Codec(ck.BLOCK_NUMBER, "ushort"),
        Codec(ck.BLOCK_TEXT, "long_string"),
    ]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        block_number = kwargs[ck.BLOCK_NUMBER]
        block_text = kwargs[ck.BLOCK_TEXT]

        parent.file_block_bank[block_number] = block_text

        # TODO: Downlink acknowledgment with block number, or downlink block numbers on request


class get_file_blocks_info(Command):
    """Downlink checksum of file blocks and any missing block numbers"""

    id = CommandEnum.GetFileBlocksInfo
    uplink_codecs = [Codec(ck.TOTAL_BLOCKS, "ushort")]
    downlink_codecs = [Codec(dk.CHECKSUM, "string"), Codec(dk.MISSING_BLOCKS, "string")]

    def _method(self, parent: MainSatelliteThread, **kwargs) -> Dict[str, str]:
        time.sleep(15)  # For testing only

        total_blocks = kwargs[ck.TOTAL_BLOCKS]
        full_file_text = ""
        missing_blocks = ""

        for i in range(total_blocks):

            try:
                block = parent.file_block_bank[i]
                full_file_text += block

            except KeyError:
                missing_blocks += str(i) + ","

        checksum = hashlib.md5(full_file_text.encode("utf-8")).hexdigest()

        return {dk.CHECKSUM: checksum, dk.MISSING_BLOCKS: missing_blocks}


class activate_file(Command):
    id = CommandEnum.ActivateFile
    uplink_codecs = [Codec(ck.TOTAL_BLOCKS, "ushort")]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        file_path = params.FILE_UPDATE_PATH
        total_blocks = kwargs[ck.TOTAL_BLOCKS]

        assert total_blocks == len(parent.file_block_bank)

        full_file_text = ""

        # Assemble file from blocks
        for i in range(total_blocks):
            full_file_text += parent.file_block_bank[i]

        # Create backup with the original if the file already exists
        if os.path.exists(file_path):
            original_file = open(file_path, "r")
            original_file_lines = original_file.readlines()
            backup_name = file_path[: file_path.index(".py")] + "_backup.py"
            backup_file = open(backup_name, "w")
            backup_file.writelines(original_file_lines)

        # Opens target file, creates one with the given path if it doesn't exist yet
        original_file = open(file_path, "w")

        # Write chained file blocks to the target file path
        original_file.seek(0)
        original_file.write(full_file_text)

        parent.file_block_bank = {}


class print_some_string(Command):
    id = CommandEnum.LongString
    uplink_codecs = [Codec("some_number", "float"), Codec("long_string", "string")]
    downlink_codecs = []

    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        number = kwargs["some_number"]
        string = kwargs["long_string"]

        logging.info(number)
        logging.info(string)


class nemo_write_register(Command):
    id = CommandEnum.NemoWriteRegister
    uplink_codecs = [Codec(ck.REG_ADDRESS, "uint8"), Codec(ck.REG_VALUE, "uint8")]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            reg_address = kwargs[ck.REG_ADDRESS]
            values = [kwargs[ck.REG_VALUE]]

            parent.nemo_manager.write_register(reg_address, values)

        else:
            logging.error(
                "CMD: nemo_write_register() failed, nemo_manager not initialized"
            )


class nemo_read_register(Command):
    id = CommandEnum.NemoReadRegister
    uplink_codecs = [Codec(ck.REG_ADDRESS, "uint8"), Codec(ck.REG_SIZE, "uint8")]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            reg_address = kwargs[ck.REG_ADDRESS]
            size = kwargs[ck.REG_SIZE]
            parent.nemo_manager.read_register(reg_address, size)

        else:
            logging.error(
                "CMD: nemo_read_register() failed, nemo_manager not initialized"
            )


class nemo_set_config(Command):
    id = CommandEnum.NemoSetConfig
    uplink_codecs = [
        Codec(ck.DET_ENABLE_UINT8, "uint8"),
        Codec(ck.DET0_BIAS_UINT8, "uint8"),
        Codec(ck.DET1_BIAS_UINT8, "uint8"),
        Codec(ck.DET0_THRESHOLD_UINT8, "uint8"),
        Codec(ck.DET1_THRESHOLD_UINT8, "uint8"),
        Codec(ck.RATE_WIDTH_MIN, "uint8"),
        Codec(ck.RATE_WIDTH_MAX, "uint8"),
        Codec(ck.BIN_WIDTH, "uint8"),
        Codec(ck.BIN_0_MIN_WIDTH, "uint8"),
        Codec(ck.RATE_INTERVAL, "uint8"),
        Codec(ck.VETO_THRESHOLD_MIN, "uint8"),
        Codec(ck.VETO_THRESHOLD_MAX, "uint8"),
        Codec(ck.CONFIG_WRITE_PERIOD, "uint8"),
        Codec(ck.CONFIG_ROTATE_PERIOD, "uint8"),
        Codec(ck.DATE_WRITE_PERIOD, "uint8"),
        Codec(ck.RATE_DATA_ROTATE_PERIOD, "int"),
        Codec(ck.HISTOGRAM_ROTATE_PERIOD, "int"),
    ]

    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            parent.nemo_manager.set_config(**kwargs)

        else:
            logging.error("CMD: nemo_set_config() failed, nemo_manager not initialized")


class nemo_power_off(Command):
    id = CommandEnum.NemoPowerOff
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            parent.nemo_manager.power_off()
        else:
            logging.error("CMD: nemo_power_off() failed, nemo_manager not initialized")


class nemo_power_on(Command):
    id = CommandEnum.NemoPowerOn
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            parent.nemo_manager.power_on()

        else:
            logging.error("CMD: nemo_power_on() failed, nemo_manager not initialized")


class nemo_reboot(Command):
    id = CommandEnum.NemoReboot
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            parent.nemo_manager.reboot()

        else:
            logging.error("CMD: nemo_reboot() failed, nemo_manager not initialized")


class nemo_process_rate_data(Command):
    id = CommandEnum.NemoProcessRateData
    uplink_codecs = []
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:

        if parent.nemo_manager is not None:
            t_start = kwargs[ck.T_START]
            t_stop = kwargs[ck.T_STOP]
            decimation_factor = kwargs[ck.DECIMATION_FACTOR]

            parent.nemo_manager.process_rate_data(t_start, t_stop, decimation_factor)

        else:
            logging.error(
                "CMD: nemo_process_rate_data() failed, nemo_manager not initialized"
            )


class nemo_process_histograms(Command):
    id = CommandEnum.NemoProcessHistograms
    uplink_codecs = [
        Codec(ck.T_START, "int"),
        Codec(ck.T_STOP, "int"),
        Codec(ck.DECIMATION_FACTOR, "uint8"),
    ]
    downlink_codecs = []

    def _method(self, parent: MainSatelliteThread, **kwargs) -> None:
        if parent.nemo_manager is not None:
            t_start = kwargs[ck.T_START]
            t_stop = kwargs[ck.T_STOP]
            decimation_factor = kwargs[ck.DECIMATION_FACTOR]

            parent.nemo_manager.process_histograms(t_start, t_stop, decimation_factor)

        else:
            logging.error(
                "CMD: nemo_process_histograms() failed, nemo_manager not initialized"
            )


def run_shell_command(cmd: str) -> int:
    logging.info(f"Running {cmd}")
    output = subprocess.run(cmd, shell=True)
    return output.returncode


SHELL_CODEC = [Codec(ck.CMD, "string")]
RETURN_CODEC = [Codec(dk.RETURN_CODE, "uint8")]


class shellCommand(Command):
    id = CommandEnum.ShellCommand
    uplink_codecs = SHELL_CODEC
    downlink_codecs = RETURN_CODEC

    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Dict[str, Union[float, int]]:
        cmd: str = cast("str", kwargs.get(ck.CMD))
        return_code = run_shell_command(cmd)
        return {dk.RETURN_CODE: return_code}


class sudoCommand(Command):
    """Same as shell_command, but prepends 'sudo ' to the command"""

    id = CommandEnum.SudoCommand
    uplink_codecs = SHELL_CODEC
    downlink_codecs = RETURN_CODEC

    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Dict[str, Union[float, int]]:
        cmd: str = cast("str", kwargs.get(ck.CMD))
        command = "sudo " + cmd
        return_code = run_shell_command(command)
        return {dk.RETURN_CODE: return_code}


class picberry(Command):
    id = CommandEnum.Picberry
    uplink_codecs = SHELL_CODEC
    downlink_codecs = RETURN_CODEC

    def _method(
        self, parent: Optional[MainSatelliteThread] = None, **kwargs
    ) -> Dict[str, Union[float, int]]:
        cmd: str = cast("str", kwargs.get(ck.CMD))
        base_command = "sudo picberry --gpio=20,21,16 --family=pic24fjxxxgb2xx "
        return_code = run_shell_command(base_command + cmd)

        return {dk.RETURN_CODE: return_code}


class exec_py_file(Command):
    id = CommandEnum.ExecPyFile
    uplink_codecs = [Codec(ck.FNAME, "string")]
    downlink_codecs = []

    def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> None:
        filename: str = cast("str", kwargs[ck.FNAME])
        filename += ".py"
        logging.debug(f"CWD: {os.getcwd()}")
        exec(open(filename).read())


class set_param(Command):
    id = CommandEnum.SetParam
    uplink_codecs = [
        Codec(ck.NAME, "string"),
        Codec(ck.VALUE, "double"),
        Codec(ck.HARD_SET, "bool"),
    ]

    downlink_codecs = [Codec(dk.VALUE, "double")]

    def _method(self, parent: MainSatelliteThread, **kwargs):
        """Changes the values of a parameter in utils/parameters.py or .json if hard_set"""
        name = kwargs[ck.NAME]
        value = kwargs[ck.VALUE]
        hard_set = kwargs[ck.HARD_SET]
        initial_value = parameter_utils.set_parameter(name, value, hard_set)

        logging.info(f"Changed constant {name} from {initial_value} to {value}")

        return {dk.VALUE: value}


GomConf1Codecs = [
    Codec(GomConfKwargs.PPT_MODE, "uint8"),
    Codec(GomConfKwargs.BATTHEATERMODE, "bool"),
    Codec(GomConfKwargs.BATTHEATERLOW, "uint8"),
    Codec(GomConfKwargs.BATTHEATERHIGH, "uint8"),
    Codec(GomConfKwargs.OUTPUT_NORMAL1, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL2, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL3, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL4, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL5, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL6, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL7, "bool"),
    Codec(GomConfKwargs.OUTPUT_NORMAL8, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE1, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE2, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE3, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE4, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE5, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE6, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE7, "bool"),
    Codec(GomConfKwargs.OUTPUT_SAFE8, "bool"),
    Codec(GomConfKwargs.OUTPUT_ON_DELAY, "ushort"),
    Codec(GomConfKwargs.OUTPUT_OFF_DELAY, "ushort"),
    Codec(GomConfKwargs.VBOOST1, "ushort"),
    Codec(GomConfKwargs.VBOOST2, "ushort"),
    Codec(GomConfKwargs.VBOOST3, "ushort"),
]


class set_gom_conf1(Command):
    id = CommandEnum.GomConf1Set
    uplink_codecs = GomConf1Codecs
    downlink_codecs = GomConf1Codecs

    def _method(
        self, parent: MainSatelliteThread, **kwargs
    ) -> Optional[Dict[str, int]]:
        new_config = gom_util.eps_config_from_dict(**kwargs)
        logging.info("New config to be set:")
        ps.displayConfig(new_config)

        if parent.gom is not None:
            try:
                parent.gom.driver.config_set(new_config)
                updated_config: ps.eps_config_t = parent.gom.driver.config_get()
                new_config_dict = gom_util.dict_from_eps_config(updated_config)
                return new_config_dict

            except Exception as e:
                logging.error(e, exc_info=True)
                logging.error("Could not set new gom config")
        else:
            logging.warning("Can't talk to Gom P31u")


class get_gom_conf1(Command):
    id = CommandEnum.GomConf1Get
    uplink_codecs = []
    downlink_codecs = GomConf1Codecs

    def _method(
        self, parent: MainSatelliteThread, **kwargs
    ) -> Optional[Dict[str, int]]:
        if parent.gom is not None:
            current_config: ps.eps_config_t = cast(
                "ps.eps_config_t", parent.gom.get_health_data(level="config")
            )
            ps.displayConfig(current_config)
            current_config_dict = gom_util.dict_from_eps_config(current_config)
            return current_config_dict
        else:
            logging.warning("Can't talk to Gom P31u")


GomConf2Codecs = [
    Codec(GomConfKwargs.MAX_VOLTAGE, "ushort"),
    Codec(GomConfKwargs.NORM_VOLTAGE, "ushort"),
    Codec(GomConfKwargs.SAFE_VOLTAGE, "ushort"),
    Codec(GomConfKwargs.CRIT_VOLTAGE, "ushort"),
]


class set_gom_conf2(Command):
    id = CommandEnum.GomConf2Set
    uplink_codecs = GomConf2Codecs
    downlink_codecs = GomConf2Codecs

    def _method(
        self, parent: MainSatelliteThread, **kwargs
    ) -> Optional[Dict[str, int]]:
        if parent.gom is not None:
            new_conf2 = gom_util.eps_config2_from_dict(kwargs)
            parent.gom.driver.config2_set(new_conf2)
            parent.gom.driver.config2_cmd(2)

            return gom_util.dict_from_eps_config2(parent.gom.driver.config2_get())
        else:
            logging.warning("Can't talk to Gom P31u")


class get_gom_conf2(Command):
    id = CommandEnum.GomConf2Get
    uplink_codecs = []
    downlink_codecs = GomConf2Codecs

    def _method(
        self, parent: MainSatelliteThread, **kwargs
    ) -> Optional[Dict[str, int]]:
        if parent.gom is not None:
            current_conf2 = cast(
                "ps.eps_config2_t", parent.gom.get_health_data(level="config2")
            )
            ps.displayConfig2(current_conf2)
            current_config2_dict = gom_util.dict_from_eps_config2(current_conf2)
            return current_config2_dict
        else:
            logging.warning("Can't talk to Gom P31u")


COMMAND_LIST: List[Command] = [
    get_gom_conf2(),
    get_gom_conf1(),
    set_gom_conf1(),
    set_gom_conf2(),
    set_param(),
    separation_test(),
    set_opnav_interval(),
    exit_low_batt_thresh(),
    acs_pulse_timing(),
    critical_telem(),
    basic_telem(),
    detailed_telem(),
    electrolysis(),
    ignore_low_batt(),
    schedule_maneuver(),
    reboot(),
    cease_comms(),
    set_system_time(),
    get_param(),
    power_cycle(),
    reboot_gom(),
    gom_outputs(),
    pi_shutdown(),
    set_file_to_update(),
    add_file_block(),
    get_file_blocks_info(),
    activate_file(),
    print_some_string(),
    nemo_write_register(),
    nemo_read_register(),
    nemo_set_config(),
    nemo_power_off(),
    nemo_power_on(),
    nemo_reboot(),
    nemo_process_rate_data(),
    nemo_process_histograms(),
    shellCommand(),
    sudoCommand(),
    picberry(),
    exec_py_file(),
    BootSwitch(),
    RestartSwitch(),
    NormalSwitch(),
    LowBatterySwitch(),
    SafetySwitch(),
    OpnavSwitch(),
    ManeuverSwitch(),
    SensorSwitch(),
    TestSwitch(),
    CommsSwitch(),
    CommandSwitch(),
    AttitudeAdjustmentSwitch(),
    run_opnav(),
]

COMMAND_DICT: Dict[int, Command] = {command.id: command for command in COMMAND_LIST}


# DEPRECATED Commands: OLD CODE, UNUSED, MAYBE WORTH BRINGING BACK?

#     def edit_file_at_line(self, **kwargs):

#         file_path = FLIGHT_SOFTWARE_PATH + kwargs['file_path']
#         line_number = kwargs['line_number']
#         new_line = kwargs['new_line']

#         # Open and copy file
#         original_file = open(file_path, 'r+')
#         original_file_lines = original_file.readlines()
#         new_file_lines = original_file_lines[:]

#         # Modify copy at designated line
#         new_file_lines[line_number] = new_line + ' \n'

#         # Write copy onto original file and original file into a backup
#         backup_file = open('backup_' + file_path, 'w')
#         backup_file.writelines(original_file_lines)
#         original_file.writelines(new_file_lines)

# class insert_line_in_file(Command):
#     id = CommandEnum.Insert
#     uplink_codecs = [Codec(TOTAL_BLOCKS, "ushort")]
#     downlink_codecs = [Codec(CHECKSUM, "string"),
#                      Codec(MISSING_BLOCKS, "string")]

#     def _method(self, parent: Optional[MainSatelliteThread] = None, **kwargs) -> Dict:
#         line_number = kwargs['line_number']
#         new_line = kwargs['new_line']
#         file_path = params.FILE_UPDATE_PATH

#         # Get original file contents
#         original_file = open(file_path, 'r+')
#         my_file_lines = original_file.readlines()
#         pre_contents = my_file_lines[:line_number]
#         post_contents = my_file_lines[line_number:]

#         # Write new line into file
#         original_file.seek(0)
#         original_file.writelines(
#             pre_contents + [new_line + ' \n'] + post_contents)

# def switch(self):
#         logging.critical("Manual FM switch commanded")

#     def adc_test(self):
#         # tests integration of ADC into the rest of the FSW
#         logging.info(
#             "Cold junction temperature for gyro sensor in Celsius:")
#         logging.info(parent.adc.get_gyro_temp())

#         logging.info(
#             f"Pressure: {parent.adc.read_pressure()} psi")
#         logging.info(
#             f"Temperature: {parent.adc.read_temperature()} deg C")

#         logging.info("Conversion sanity check: 25.6 degrees")
#         logging.info(parent.adc.convert_volt_to_temp(
#             parent.adc.convert_temp_to_volt(25.6)))
#         logging.info("Conversion sanity check: 2.023 mV")
#         logging.info(parent.adc.convert_temp_to_volt(
#             parent.adc.convert_volt_to_temp(2.023)))

#     def rtc_test(self):
#         logging.info(
#             f"Oscillator Disabled: {parent.rtc.ds3231.disable_oscillator}")
#         logging.info(f"RTC Temp: {parent.rtc.get_temp()}")
#         logging.info(f"RTC Time: {parent.rtc.get_time()}")
#         # time.sleep(1)
#         logging.info("Setting RTC time to 1e9")
#         parent.rtc.set_time(int(1e9))
#         logging.info("New RTC Time: {parent.rtc.get_time()}")
#         # time.sleep(1)
#         logging.info("Incrementing RTC Time by 5555 seconds")
#         parent.rtc.increment_rtc(5555)
#         logging.info(
#             f"New RTC Time: {parent.rtc.get_time()}")
#         logging.info("Disabling Oscillator, waiting 10 seconds")
#         # parent.rtc.disable_oscillator()
#         time.sleep(10)
#         logging.info(
#             f"RTC Time after disabling oscillator: {parent.rtc.get_time()}")
#         logging.info("Enabling Oscillator, waiting 10 seconds")
#         # parent.rtc.enable_oscillator()
#         time.sleep(10)
#         logging.info(
#             f"RTC Time after re-enabling oscillator: {parent.rtc.get_time()}")
#         logging.info("Disabling Oscillator")
#         # parent.rtc.disable_oscillator()
#         parent.handle_sigint(None, None)
