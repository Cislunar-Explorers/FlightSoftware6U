from utils.constants import CommandEnum
from communications.command_definitions import COMMAND_LIST
import unittest


class CommandDefinitionTest(unittest.TestCase):
    def test_flight_commands(self):
        """Compares the command IDs that are defined in command_definition's command lists with the IDs defined in
        the CommandEnum in constants.py"""
        # want to make sure that all command IDs defined in utils.constants matches what's in command_definitions

        error_msg = ""
        all_enum_command_ids = list(map(int, CommandEnum))
        all_command_ids = [command.id for command in COMMAND_LIST]

        try:
            self.assertCountEqual(all_enum_command_ids, all_command_ids)
        except AssertionError:
            commands_not_in_both = set(
                all_enum_command_ids).symmetric_difference(all_command_ids)
            error_msg += f"Commands defined in CommandEnum not consistent with commands in command_definitions: {commands_not_in_both}\n"

        if error_msg != "":
            raise AssertionError(error_msg)

    def test_unique_command_ids(self):
        """Verifies that the command IDs as defined in command_definitions are all unique"""
        all_command_ids = [command.id for command in COMMAND_LIST]

        if len(all_command_ids) != len(set(all_command_ids)):
            raise AssertionError(
                f"IDs used more than once: {[id for id in all_command_ids if all_command_ids.count(id) > 1]}")


if __name__ == '__main__':
    unittest.main()
