from communications.commands import CommandHandler
from communications.command_definitions import CommandDefinitions
from flight_modes.flight_mode_factory import build_flight_mode
from utils.constants import FMEnum, BootCommandEnum, NormalCommandEnum, SafetyCommandEnum, LowBatterySafetyCommandEnum, \
    TestCommandEnum, CommandCommandEnum

command_enums = [BootCommandEnum, NormalCommandEnum, SafetyCommandEnum, LowBatterySafetyCommandEnum, TestCommandEnum,
                 CommandCommandEnum]
command_ids = [FMEnum.Boot.value, FMEnum.Normal.value, FMEnum.Safety.value, FMEnum.LowBatterySafety.value,
               FMEnum.TestMode.value, FMEnum.Command.value]


def testFlightCommands():
    ch = CommandHandler()
    ch.register_commands()
    cd = CommandDefinitions(None)
    all_modes = list(map(int, FMEnum))

    i = 0
    for command_enum in command_enums:
        command_fm = command_ids[i]
        i += 1
        for command_id in command_enum:
            assert command_fm in cd.COMMAND_DICT
            print(command_id.value)
            print(cd.COMMAND_DICT[command_fm])
            assert command_id.value in cd.COMMAND_DICT[command_fm]


if __name__ == '__main__':
    testFlightCommands()
