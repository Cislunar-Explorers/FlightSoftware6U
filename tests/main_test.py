import unittest
from main import MainSatelliteThread


class MyTestCase(unittest.TestCase):
    def test_main_satellite_thread_init(self):
        m = MainSatelliteThread()
        self.assertIsNotNone(m.flight_mode)
        self.assertIsNotNone(m.FMQueue)
        self.assertIsNotNone(m.telemetry)
        self.assertIsNotNone(m.command_definitions)
        self.assertIsNotNone(m.command_handler)

        # Assert all sensors are None
        self.assertIsNone(m.gom)
        self.assertIsNone(m.adc)
        self.assertIsNone(m.gyro)
        self.assertIsNone(m.rtc)
        self.assertIsNone(m.radio)
        self.assertIsNone(m.mux)
        self.assertIsNone(m.camera)
        self.assertIsNone(m.nemo_manager)


if __name__ == '__main__':
    unittest.main()
