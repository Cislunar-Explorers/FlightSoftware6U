import unittest
from communications import commands
from utils import constants
import bitstring


class BitFlips(unittest.TestCase):
    """Tests that the command handler is able to reject commands that have a bit flipped. This test emulates
    generating and sending packets from the ground, having one or more of the bits in the packet getting flipped,
    and then having that corrupted packet being received by the spacecraft. """

    def setUp(self) -> None:
        self.command_handler = commands.CommandHandler()

    def bit_flip_tester(self, flip_bit: bool):
        COUNTER = 0
        FMID = constants.FMEnum.Normal.value
        cmd_id = constants.NormalCommandEnum.SetParam.value
        command_kwargs = {"name": "OPNAV_INTERVAL", "value": 30.0, "hard_set": True}
        bytes_to_transmit = self.command_handler.pack_command(COUNTER, FMID, cmd_id, **command_kwargs)
        bits_to_transmit = int.from_bytes(bytes_to_transmit, 'big')  # bits that get transmitted

        for i in range(len(bin(bits_to_transmit)[2:])):
            received_bits = bits_to_transmit ^ flip_bit << i  # emulate bitflip at position i using XOR
            received_bytes = received_bits.to_bytes(len(bytes_to_transmit), 'big')  # repack bitflipped data into bytes
            mac, counter, mode, command_id, arg_data = self.command_handler.unpack_command(received_bytes)
            self.assertEqual(mac, constants.MAC)
            self.assertEqual(counter, COUNTER)
            self.assertEqual(mode, FMID)
            self.assertEqual(command_id, cmd_id)
            self.assertEqual(arg_data, command_kwargs)

    def test_no_bit_flips(self):
        self.bit_flip_tester(False)

    def test_single_bit_flip(self):
        self.bit_flip_tester(True)


if __name__ == '__main__':
    unittest.main()
