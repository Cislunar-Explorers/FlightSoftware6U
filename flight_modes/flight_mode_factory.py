from utils.constants import FMEnum
from utils.exceptions import UnknownFlightModeException

from flight_modes.attitude_adjustment import AAMode
from flight_modes.flight_mode import (
    CommandMode,
    CommsMode,
    NormalMode,
    SafeMode,
    SensorMode,
    TestMode,
)
from flight_modes.low_battery import LowBatterySafetyMode
from flight_modes.maneuver_flightmode import ManeuverMode
from flight_modes.opnav_flightmode import OpNavMode
from flight_modes.restart_reboot import BootUpMode, RestartMode

FLIGHT_MODE_DICT = {
    FMEnum.Boot.value: BootUpMode,
    FMEnum.Restart.value: RestartMode,
    FMEnum.Normal.value: NormalMode,
    FMEnum.LowBatterySafety.value: LowBatterySafetyMode,
    FMEnum.Safety.value: SafeMode,
    FMEnum.OpNav.value: OpNavMode,
    FMEnum.Maneuver.value: ManeuverMode,
    FMEnum.SensorMode.value: SensorMode,
    FMEnum.TestMode.value: TestMode,
    FMEnum.CommsMode.value: CommsMode,
    FMEnum.Command.value: CommandMode,
    FMEnum.AttitudeAdjustment.value: AAMode,
}


def build_flight_mode(main, fm_id, **kwargs):
    try:
        cls = FLIGHT_MODE_DICT[fm_id]
        return cls(main)
    except KeyError:
        raise UnknownFlightModeException(fm_id)
