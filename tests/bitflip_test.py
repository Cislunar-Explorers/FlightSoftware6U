from utils.exceptions import DeserializationException
import unittest
from communications import command_handler
from utils import constants, log
from typing import List
import logging

DEBUG = False


class BitFlips(unittest.TestCase):
    """Tests that the command handler is able to reject commands that have a bit flipped. This test emulates
    generating and sending packets from the ground, having one or more of the bits in the packet getting flipped,
    and then having that corrupted packet being received by the spacecraft. """

    def setUp(self) -> None:
        self.command_handler = command_handler.CommandHandler()

    def bit_flip_tester(self, flip_bit: bool):
        rejected_by_mac = 0
        cmd_id = constants.CommandEnum.SetParam.value
        command_kwargs = {"name": "OPNAV_INTERVAL",
                          "value": 30.0, "hard_set": True}
        bytes_to_transmit = self.command_handler.pack_command(
            cmd_id, **command_kwargs)
        bits_to_transmit = int.from_bytes(
            bytes_to_transmit, 'big')  # bits that get transmitted

        bad_indecies: List[int] = []
        for i in range(len(bin(bits_to_transmit)[2:])):
            # emulate bitflip at position i using XOR
            received_bits = bits_to_transmit ^ flip_bit << i
            received_bytes = received_bits.to_bytes(
                len(bytes_to_transmit), 'big')  # repack bitflipped data into bytes
            try:
                command, arg_data = self.command_handler.unpack_link(
                    received_bytes)
                # self.assertEqual(counter, COUNTER)
                self.assertEqual(command.id, cmd_id)
                self.assertEqual(arg_data, command_kwargs)
            except AssertionError as ae:
                # if the assertions fail, the data was corrupted and we couldn't do anything about it
                bad_indecies.append(i)
                if DEBUG:
                    log.log_error(ae, logging.error)
            except DeserializationException as cue:
                # if the command unpacker catches the error, we're good.
                if DEBUG:
                    log.log_error(cue, logging.warning)
                    rejected_by_mac += 1
            except Exception as e:
                # if something else fails, that's bad
                bad_indecies.append(i)
                if DEBUG:
                    log.log_error(e, logging.error)

        if DEBUG:
            print(
                f"Rejected by MAC: {rejected_by_mac}/{len(bin(bits_to_transmit)[2:])}")
        self.assertListEqual(bad_indecies, [])

    def test_no_bit_flips(self):
        self.bit_flip_tester(False)

    def test_single_bit_flip(self):
        self.bit_flip_tester(True)


if __name__ == '__main__':
    unittest.main()
