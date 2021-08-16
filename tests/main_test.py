import unittest
from main import MainSatelliteThread


class MyTestCase(unittest.TestCase):
    def test_main_satellite_thread_init(self):
        m = MainSatelliteThread()
        self.assertIsNotNone(m.flight_mode)
        self.assertIsNotNone(m.FMQueue)
        self.assertIsNotNone(m.telemetry)


if __name__ == '__main__':
    unittest.main()
