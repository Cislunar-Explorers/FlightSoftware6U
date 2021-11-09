from queue import PriorityQueue
import pytest
from time import time

from main import MainSatelliteThread
from flight_modes.maneuver_flightmode import ManeuverMode
import utils.constants as consts
import utils.parameters as params

# to speed testing
params.GLOW_WAIT_TIME = 0.01


class FakeGOM:
    def glowplug(self, x):
        return 0

    def glowplug2(self, x):
        return 0


@pytest.fixture
def m():
    m = MainSatelliteThread()
    m.replace_flight_mode_by_id(consts.FMEnum.Maneuver)
    m.gom = FakeGOM()
    return m


def test_maneuver_selection(m, mocker):
    mocker.patch(
        "flight_modes.maneuver_flightmode.ManeuverMode.get_pressure",
        side_effect=[100, 90, 80, 70],
    )
    assert isinstance(m.maneuver_queue, PriorityQueue)
    assert isinstance(m.flight_mode, ManeuverMode)
    cur_time = time()
    m.command_definitions.schedule_maneuver(time=cur_time + 1000)
    m.command_definitions.schedule_maneuver(time=cur_time + 100)
    assert len(m.maneuver_queue.queue) == 1
    assert params.SCHEDULED_BURN_TIME == cur_time + 100
    m.flight_mode.run_mode()
    assert m.maneuver_queue.empty()
    assert params.SCHEDULED_BURN_TIME == cur_time + 1000
    m.flight_mode.run_mode()
    assert m.maneuver_queue.empty()
    assert params.SCHEDULED_BURN_TIME == -1


def test_maneuver_valid_glowplug(m):
    assert m.flight_mode.valid_glowplug(90, 100)
