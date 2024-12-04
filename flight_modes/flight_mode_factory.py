from fsw.flight_modes.flight_mode import (
    NormalMode,
    SafeMode,
    SensorMode,
    TestMode,
    CommsMode,
    OpNavMode,
    CommandMode,
)

from fsw.flight_modes.low_battery import LowBatterySafetyMode

from fsw.flight_modes.restart_reboot import RestartMode, BootUpMode
from fsw.flight_modes.maneuver_flightmode import ManeuverMode
from fsw.flight_modes.attitude_adjustment import AAMode
from fsw.utils.constants import FMEnum
from fsw.utils.exceptions import UnknownFlightModeException

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
