from flight_modes.flight_mode import (
    NormalMode,
    LowBatterySafetyMode,
    SafeMode,
    ManeuverMode,
    SensorMode,
    TestMode,
    CommsMode,
)

#from flight_modes.restart_reboot import RestartMode, BootUpMode
from .opnav_flightmode import OpNavMode
from utils.constants import FMEnum
from utils.exceptions import UnknownFlightModeException

FLIGHT_MODE_DICT = {
    #FMEnum.Boot.value: BootUpMode,
    #FMEnum.Restart.value: RestartMode,
    FMEnum.Normal.value: NormalMode,
    FMEnum.LowBatterySafety.value: LowBatterySafetyMode,
    FMEnum.Safety.value: SafeMode,
    FMEnum.OpNav.value: OpNavMode,
    FMEnum.Maneuver.value: ManeuverMode,
    FMEnum.SensorMode.value: SensorMode,
    FMEnum.TestMode.value: TestMode,
    FMEnum.CommsMode.value: CommsMode,
}


def build_flight_mode(main, fm_id, **kwargs):
    try:
        cls = FLIGHT_MODE_DICT[fm_id]
        return cls(main)
    except KeyError:
        raise UnknownFlightModeException(fm_id)
