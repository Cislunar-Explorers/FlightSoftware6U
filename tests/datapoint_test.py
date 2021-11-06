import unittest
from communications.datapoint import DataPoint
from math import tan

class DataPointTest(unittest.TestCase):
    def test_bool_point(self):
        test_bool = DataPoint("TEST_BOOL", "bool")

        value = True
        packed = test_bool.pack(value)
        unpacked = test_bool.unpack(packed)

        self.assertEqual(value, unpacked[test_bool.name])

        value = False
        packed = test_bool.pack(value)
        unpacked = test_bool.unpack(packed)

        self.assertEqual(value, unpacked[test_bool.name])

    def test_uint8_point(self):
        test_uint8 = DataPoint("TEST_UINT8", 'uint8')

        for value in range(255):
            packed = test_uint8.pack(value)
            unpacked = test_uint8.unpack(packed)
            self.assertEqual(value, unpacked[test_uint8.name])

    def test_short_point(self):
        test_short = DataPoint("TEST_SHORT", 'short')

        for value in range(0, 2*32_767):
            packed = test_short.pack(value)
            unpacked = test_short.unpack(packed)
            self.assertEqual(value, unpacked[test_short.name])

    def test_int_point(self):
        test_int = DataPoint("TEST_INT", 'int')

        for value in range(0, 2*32_767-1):
            packed = test_int.pack(value)
            unpacked = test_int.unpack(packed)
            self.assertEqual(value, unpacked[test_int.name])

    def test_long_point(self):
        test_long = DataPoint("TEST_LONG", 'long')

        for value in range(0, 4*32_767):
            packed = test_long.pack(value)
            unpacked = test_long.unpack(packed)
            self.assertEqual(value, unpacked[test_long.name])

    def test_float_point(self):
        test_float = DataPoint("TEST_FLOAT", 'float')

        num = 100
        for value in [float(tan(x*(3.14159/2)/num)) for x in range(-num, num)[1:-1]]:
            packed = test_float.pack(value)
            unpacked = test_float.unpack(packed)
            self.assertAlmostEqual(value, unpacked[test_float.name], places=5)

    def test_double_point(self):
        test_double = DataPoint("TEST_DOUBLE", 'double')

        num = 10000
        for value in [float(tan(x*(3.14159/2)/num)) for x in range(-num, num)[1:-1]]:
            packed = test_double.pack(value)
            unpacked = test_double.unpack(packed)
            self.assertEqual(value, unpacked[test_double.name])


if __name__ == '__main__':
    unittest.main()
