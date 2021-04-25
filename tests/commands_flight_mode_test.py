from main import MainSatelliteThread
from communications.command_definitions import CommandDefinitions
from utils.constants import FMEnum, BootCommandEnum, RestartCommandEnum, NormalCommandEnum, LowBatterySafetyCommandEnum, \
    SafetyCommandEnum, OpNavCommandEnum, ManeuverCommandEnum, SensorsCommandEnum, CommsCommandEnum, TestCommandEnum, \
    CommandCommandEnum

from flight_modes.flight_mode_factory import FLIGHT_MODE_DICT

command_enums = [BootCommandEnum, RestartCommandEnum, NormalCommandEnum, LowBatterySafetyCommandEnum,
                 SafetyCommandEnum, OpNavCommandEnum, ManeuverCommandEnum, SensorsCommandEnum, TestCommandEnum,
                 CommsCommandEnum, CommandCommandEnum]


def testFlightCommands():
    cd = CommandDefinitions(MainSatelliteThread())
    # all_modes = list(map(int, FMEnum))

    zipped_enums_command_dicts = list(zip(command_enums, cd.COMMAND_DICT.values()))
    # want to make sure that all command IDs defined in utils.constants matches what's in command_definitions

    error_msg = ""

    for enum, command_dict in zipped_enums_command_dicts:
        all_enum_commands = list(map(int, enum))
        all_command_ids = list(command_dict.keys())
        all_enum_commands = sorted(all_enum_commands)
        all_command_ids = sorted(all_command_ids)

        try:
            assert all_enum_commands == all_command_ids
        except AssertionError:
            commands_not_in_both = set(all_enum_commands).symmetric_difference(all_command_ids)
            error_msg += f"Commands defined in {enum} not consistent with utils.constants: {commands_not_in_both}\n"

    if error_msg != "":
        raise AssertionError(error_msg)


def testFlightCodecs():
    flight_mode_objs = list(FLIGHT_MODE_DICT.values())
    fm_enums_zipped = list(zip(flight_mode_objs, command_enums))
    # print(fm_enums_zipped)
    error_msg = ""
    for fm_obj, command_enum in fm_enums_zipped:
        command_enum_list = list(map(int, command_enum))
        command_codecs_list = list(fm_obj.command_codecs.keys())

        command_enum_list = sorted(command_enum_list)
        command_codecs_list = sorted(command_codecs_list)
        try:
            commands_not_in_both = set(command_enum_list).symmetric_difference(command_codecs_list)
            assert len(commands_not_in_both) == 0
        except AssertionError:
            error_msg += f"Command codec definition inconsistency. Check FM and constants: FMID: {fm_obj.flight_mode_id}, CID: {commands_not_in_both}\n"

    if error_msg != '':
        raise AssertionError(error_msg)


def testFlightArgTypes():
    flight_mode_objs = list(FLIGHT_MODE_DICT.values())
    all_command_arg_types = []
    for fm_obj in flight_mode_objs:
        all_command_arg_types.extend(list(fm_obj.command_arg_types.keys()))
    all_arg_type_set = set(all_command_arg_types)

    assert len(all_command_arg_types) == len(all_arg_type_set)


if __name__ == '__main__':
    testFlightCommands()
    testFlightCodecs()
    testFlightArgTypes()
