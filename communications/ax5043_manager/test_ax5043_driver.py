import unittest
from communications.ax5043_manager.ax5043_driver import *
from communications.ax5043_manager.mock_ax5043_driver import MockAx5043, rst_values


class TestChunk(unittest.TestCase):
    def test_from_bytes(self):
        buf = bytearray(
            [
                0x55,
                0x80,
                0x80,
                0xE1,
                0x05,
                0x03,
                0xCA,
                0xFE,
                0xBA,
                0xBE,
                0x31,
                0xBD,
                0x73,
                0x00,
                0x00,
                0x00,
            ]
        )

        (c, buf) = Chunk.from_bytes(buf)
        self.assertTrue(isinstance(c, Antrssi2Chunk))
        self.assertEqual(c.rssi, -128)
        self.assertEqual(c.bgndnoise, -128)

        (c, buf) = Chunk.from_bytes(buf)
        self.assertTrue(isinstance(c, DataChunk))
        self.assertEqual(c.flags, 0x03)
        self.assertEqual(c.data, bytearray([0xCA, 0xFE, 0xBA, 0xBE]))

        (c, buf) = Chunk.from_bytes(buf)
        self.assertTrue(isinstance(c, RssiChunk))
        self.assertEqual(c.rssi, -67)

        (c, buf) = Chunk.from_bytes(buf)
        self.assertTrue(isinstance(c, RffreqoffsChunk))
        self.assertEqual(c.rffreqoffs, 0)

    def test_reg_dump(self):
        mock_radio = MockAx5043()
        reg_dict = mock_radio.reg_dump()
        fixed_rst_vals = {reg.name: rst_values[reg.value] for reg in Reg}
        self.assertCountEqual(reg_dict, fixed_rst_vals)


if __name__ == "__main__":
    unittest.main()
