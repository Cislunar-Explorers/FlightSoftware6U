import unittest
from utils.constants import CommandEnum, NAME, VALUE, HARD_SET, FMEnum
from main import MainSatelliteThread
from communications.command_handler import CommandHandler
import utils.parameters as params
from communications.ax5043_manager.mock_ax5043_driver import MockAx5043
from communications.satellite_radio import MockRadio


class CommandTest(unittest.TestCase):
    """This test simulates the full command flow we expect to see in flight. This includes:
        - Packing a command on the ground station and sending it
        - Receiving that command on the spacecraft and unpacking it
        - Executing the received command, and collecting the telemetry outputs from the command
        - Packing the telemetry, bit inflating and downlinking it
        - Receiving the inflated telemetry and unpacking ut"""

    def setUp(self) -> None:
        self.ground_station = CommandHandler(None)
        self.sat = MainSatelliteThread()

    def test_commands(self):
        new_value = 33

        command_0 = self.ground_station.pack_command(CommandEnum.Normal)
        command_1 = self.ground_station.pack_command(
            CommandEnum.SetParam,
            **{NAME: "OPNAV_INTERVAL", VALUE: new_value, HARD_SET: False}
        )

        # emulate sending command to satellite
        self.sat.command_queue.put(command_0)
        self.sat.execute_commands()  # satellite executes commands
        self.assertEqual(
            self.sat.flight_mode.flight_mode_id, FMEnum.Normal.value
        )  # verify that it worked

        # emulate sending command to satellite
        self.sat.command_queue.put(command_1)
        self.sat.execute_commands()  # satellite executes commands
        # verify that it worked
        self.assertEqual(params.OPNAV_INTERVAL, new_value)

        # emulate downlink
        downlink = self.sat.downlink_queue.get()
        downlink_command, downlink_args = self.ground_station.unpack_telemetry(downlink)
        # print(downlink.hex())
        # print(downlink_args)
        self.assertEqual(downlink_command.id, CommandEnum.SetParam)
        self.assertEqual(downlink_args[VALUE], new_value)
        self.assertFalse(downlink_args[HARD_SET])

    def test_manual_fm_commands(self):
        """Commands the spacecraft to go into every flight mode """
        for mode in FMEnum:
            uplink = self.ground_station.pack_command(mode.value)
            self.sat.command_queue.put(uplink)
            self.sat.execute_commands()  # satellite executes commands
            self.assertEqual(
                self.sat.flight_mode.flight_mode_id, mode.value
            )  # verify flight mode

    def test_radio_register_dump(self):
        self.sat.radio = MockRadio()
        self.sat.radio.driver = MockAx5043()
        radio_command = self.ground_station.pack_command(CommandEnum.RegDump)
        self.sat.command_queue.put(radio_command)
        self.sat.execute_commands()  # satellite executes commands


if __name__ == "__main__":
    unittest.main()
