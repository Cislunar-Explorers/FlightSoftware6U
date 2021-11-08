import unittest
from utils.constants import CommandEnum, NAME, VALUE, HARD_SET, FMEnum
from main import MainSatelliteThread
from communications.command_handler import CommandHandler
import utils.parameters as params
import logging


class CommandTest(unittest.TestCase):
    def test_commands(self):
        ground_command_handler = CommandHandler(None)
        sat = MainSatelliteThread()
        new_value = 33

        command_0 = ground_command_handler.pack_command(CommandEnum.Normal)
        command_1 = ground_command_handler.pack_command(
            CommandEnum.SetParam, **{NAME: "OPNAV_INTERVAL", VALUE: new_value, HARD_SET: False})

        # emulate sending command to satellite
        sat.command_queue.put(command_0)
        sat.execute_commands()  # satellite executes commands
        self.assertEqual(sat.flight_mode.flight_mode_id,
                         FMEnum.Normal.value)  # verify that it worked

        # emulate sending command to satellite
        sat.command_queue.put(command_1)
        sat.execute_commands()  # satellite executes commands
        # verify that it worked
        self.assertEqual(params.OPNAV_INTERVAL, new_value)

        # emulate downlink
        downlink = sat.downlink_queue.get()
        downlink_command, downlink_args = ground_command_handler.unpack_telemetry(
            downlink)
        # print(downlink.hex())
        # print(downlink_args)
        self.assertEqual(downlink_command.id, CommandEnum.SetParam)
        self.assertEqual(downlink_args[VALUE], new_value)
        self.assertFalse(downlink_args[HARD_SET])
