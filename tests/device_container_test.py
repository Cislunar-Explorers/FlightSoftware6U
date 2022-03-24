import unittest
from drivers.devices import DeviceContainer


class MainThreadInitTestCase(unittest.TestCase):
    def test_connect(self):
        """Verifies that the DeviceContainer initializes and behaves as we would expect
        if nothing was connected to the computer running this test. If you are running this test
        on a Raspberry Pi with any of the sensors connected properly, this test will FAIL. This is
        intended to only be run on developer's computers and the CI, not on flight or development hardware"""
        dc = DeviceContainer()
        connected_mask = dc.connect()  # try to connect to hardware
        self.assertFalse(
            all(connected_mask.values())
        )  # everything should not be able to connect
