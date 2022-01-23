import logging
import time
from queue import PriorityQueue

import pytest
import utils.constants as consts
import utils.parameters as params
from flight_modes.maneuver_flightmode import ManeuverMode
from main import MainSatelliteThread
from drivers.gom import Gomspace

# to speed testing
params.GLOW_WAIT_TIME = 0.01


class FakeGOM(Gomspace):
    def __init__(self):
        self.pc = None

    def glowplug(self, duration, delay=0):
        logging.info("Firing glowplug 1")
        return None

    def glowplug2(self, duration, delay=0):
        logging.info("Firing glowplug 2")
        return None


@pytest.fixture
def m(mocker):
    m = MainSatelliteThread()
    m.replace_flight_mode_by_id(consts.FMEnum.Maneuver)
    m.gom = FakeGOM()
    return m


def test_maneuver_selection(m: MainSatelliteThread, mocker):
    mocker.patch(
        "flight_modes.maneuver_flightmode.ManeuverMode.get_pressure",
        side_effect=[100, 90, 80, 70],
    )
    assert isinstance(m.maneuver_queue, PriorityQueue)
    assert isinstance(m.flight_mode, ManeuverMode)
    cur_time = time.time()
    maneuver_command = m.command_handler.get_command_from_id(
        consts.CommandEnum.ScheduleManeuver
    )
    m.command_handler.run_command(
        maneuver_command, {consts.CommandKwargs.MANEUVER_TIME: cur_time + 6}
    )
    m.command_handler.run_command(
        maneuver_command, {consts.CommandKwargs.MANEUVER_TIME: cur_time + 2}
    )

    # m.command_definitions.schedule_maneuver(time=cur_time + 1000)
    # m.command_definitions.schedule_maneuver(time=cur_time + 100)
    assert len(m.maneuver_queue.queue) == 1
    assert params.SCHEDULED_BURN_TIME == cur_time + 2
    m.flight_mode.run_mode()
    assert m.maneuver_queue.empty()
    assert params.SCHEDULED_BURN_TIME == cur_time + 6
    m.flight_mode.run_mode()
    assert m.maneuver_queue.empty()
    assert params.SCHEDULED_BURN_TIME == -1


def test_maneuver_valid_glowplug(m):
    assert m.flight_mode.valid_glowplug(90, 100)
