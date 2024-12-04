"""
What is the purpose of this test?
- To verify that our beloved MainSatelliteThread object is able to initialize without any errors
- To run a quick sanity check that the attributes of the thread object are what we expect them to be
"""
import unittest
from drivers.devices import DeviceContainer
from main.main import MainSatelliteThread
from queue import Queue

from communications.command_handler import CommandHandler
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
        self.assertListEqual(m.reorientation_list, EMPTY_LIST)

        self.assertIsInstance(m.command_handler, CommandHandler)
        self.assertIsInstance(m.telemetry, Telemetry)
        self.assertIsInstance(m.flight_mode, FlightMode)

        # verify all sensor objects exist and are None
        self.assertIsInstance(m.devices, DeviceContainer)
        self.assertIsNone(m.mux)
        self.assertIsNone(m.camera)
        self.assertIsNone(m.nemo_manager)


if __name__ == "__main__":
    unittest.main()
