"""
What is the purpose of this test?
- To verify that our beloved MainSatelliteThread object is able to initialize without any errors
- To run a quick sanity check that the attributes of the thread object are what we expect them to be
"""
import unittest
from main import MainSatelliteThread
from queue import Queue

from communications.command_handler import CommandHandler
from communications.downlink import DownlinkHandler
from communications.command_definitions import CommandDefinitions
from telemetry.telemetry import Telemetry
from flight_modes.flight_mode import FlightMode

EMPTY_LIST = []


class MainThreadInitTestCase(unittest.TestCase):
    def test_main_satellite_thread_init(self):
        # verify that we can initialize a MainSatelliteThread object without errors
        m = MainSatelliteThread()

        # verify that all the object attributes are accessible and reasonable
        self.assertIsInstance(m.command_queue, Queue)
        self.assertIsInstance(m.downlink_queue, Queue)
        self.assertIsInstance(m.FMQueue, Queue)
        self.assertIsInstance(m.burn_queue, Queue)
        self.assertIsInstance(m.reorientation_queue, Queue)
        self.assertIsInstance(m.opnav_queue, Queue)

        self.assertListEqual(m.commands_to_execute, EMPTY_LIST)
        self.assertListEqual(m.downlinks_to_execute, EMPTY_LIST)
        self.assertListEqual(m.reorientation_list, EMPTY_LIST)

        self.assertIsInstance(m.command_definitions, CommandDefinitions)
        self.assertIsInstance(m.command_handler, CommandHandler)
        self.assertIsInstance(m.downlink_handler, DownlinkHandler)
        self.assertIsInstance(m.telemetry, Telemetry)
        self.assertIsInstance(m.flight_mode, FlightMode)

        # verify all sensor objects are None
        self.assertIsNone(m.gom)
        self.assertIsNone(m.gyro)
        self.assertIsNone(m.adc)
        self.assertIsNone(m.rtc)
        self.assertIsNone(m.radio)
        self.assertIsNone(m.mux)
        self.assertIsNone(m.camera)
        self.assertIsNone(m.nemo_manager)


if __name__ == '__main__':
    unittest.main()
