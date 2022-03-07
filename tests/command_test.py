import logging
import unittest
from utils.constants import CommandEnum, FMEnum, CommandKwargs as ck
from main import MainSatelliteThread
from communications.command_handler import CommandHandler
import utils.parameters as params


class CommandTest(unittest.TestCase):
    """This test simulates the full command flow we expect to see in flight. This includes:
        - Packing a command on the ground station and sending it
        - Receiving that command on the spacecraft and unpacking it
        - Executing the received command, and collecting the telemetry outputs from the command
        - Packing the telemetry, bit inflating and downlinking it
        - Receiving the inflated telemetry and unpacking ut"""

    def setUp(self) -> None:
        self.sat = MainSatelliteThread()
        self.ground_station = CommandHandler(None)

    def test_commands(self):
        new_value = 33

        command_0 = self.ground_station.pack_command(CommandEnum.Normal, {})
        command_1 = self.ground_station.pack_command(
            CommandEnum.SetParam,
            {ck.NAME: "OPNAV_INTERVAL", ck.VALUE: new_value, ck.HARD_SET: False},
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
        self.assertEqual(downlink_args[ck.VALUE], new_value)

    def test_setEMSThresh(self):
        new_earth_thresh = 0.07
        new_moon_thresh = 0.025
        new_sun_thresh = 0.06

        cmd = self.ground_station.pack_command(
            CommandEnum.SetEMSThresh,
            {
                ck.EARTH_THRESH: new_earth_thresh,
                ck.MOON_THRESH: new_moon_thresh,
                ck.SUN_THRESH: new_sun_thresh,
            },
        )

        # Checking uplink
        self.sat.command_queue.put(cmd)
        self.sat.execute_commands()
        self.assertAlmostEqual(params.EARTH_PERCENTAGE_THRESH, new_earth_thresh, 3)
        self.assertAlmostEqual(params.MOON_PERCENTAGE_THRESH, new_moon_thresh, 3)
        self.assertAlmostEqual(params.SUN_PERCENTAGE_THRESH, new_sun_thresh, 2)

        # Checking downlink
        # downlink = self.sat.downlink_queue.get()
        # downlink_command, downlink_args = self.ground_station.unpack_telemetry(downlink)
        # self.assertEqual(downlink_command.id, CommandEnum.SetEMSThresh)
        # self.assertEqual(downlink_args[ck.EARTH_THRESH], new_earth_thresh)
        # self.assertEqual(downlink_args[ck.MOON_THRESH], new_moon_thresh)
        # self.assertEqual(downlink_args[ck.SUN_THRESH], new_sun_thresh)

    def test_manual_fm_commands(self):
        """Commands the spacecraft to go into every flight mode """
        for mode in FMEnum:
            uplink = self.ground_station.pack_command(mode.value, {})
            self.sat.command_queue.put(uplink)
            self.sat.execute_commands()  # satellite executes commands
            self.assertEqual(
                self.sat.flight_mode.flight_mode_id, mode.value
            )  # verify flight mode

    def test_get_param_command(self):
        """Tests whether we can 'get' the value of every parameter thru the command structure."""
        from utils.parameter_utils import get_parameter_list, get_parameter_from_name

        param_list = get_parameter_list()
        # loop thru all parameters and get their value

        # `FILE_UPDATE_PATH` is the one parameter that's a string. all others are ints/floats
        # The get_param command can only downlink floats. Will need to def another command to downlink strings
        param_list.remove("FILE_UPDATE_PATH")

        # Dont wanna mess with the counter values
        param_list.remove("DOWNLINK_COUNTER")
        param_list.remove("UPLINK_COUNTER")

        for param_name in param_list:
            try:
                uplink = self.ground_station.pack_command(
                    CommandEnum.GetParam, {ck.NAME: param_name}
                )
                self.sat.command_queue.put(uplink)
                self.sat.execute_commands()  # satellite executes command (prints param value)
                # the command adds the parameter value to the downlink queue
                # emulate downlinking
                downlink = self.sat.downlink_queue.get()
                downlink_command, downlink_args = self.ground_station.unpack_telemetry(
                    downlink
                )

                # verify that what we downlink checks out
                self.assertEqual(downlink_command.id, CommandEnum.GetParam)
                self.assertEqual(
                    list(downlink_args.values())[0], get_parameter_from_name(param_name)
                )
                self.assertEqual(list(downlink_args.keys())[0], ck.VALUE)

            except Exception:
                logging.error(f"Parameter {param_name} failed")
                raise

    def test_counter_persistence(self):
        """Test whether the uplink and downlink counters remain consistent even if the thread stops"""
        self.test_commands()  # run the same test as before (to make sure that the counters aren't 0)

        up_before, down_before = (
            self.sat.command_handler.uplink_counter,
            self.sat.command_handler.downlink_counter,
        )
        self.sat = MainSatelliteThread()  # re-init the object
        up_after, down_after = (
            self.sat.command_handler.uplink_counter,
            self.sat.command_handler.downlink_counter,
        )

        self.assertEqual(up_before, up_after)
        self.assertEqual(down_before, down_after)

        self.test_commands()


if __name__ == "__main__":
    unittest.main()
