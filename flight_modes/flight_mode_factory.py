from .flight_mode import (
    BootUpMode,
    RestartMode,
    NormalMode,
    LowBatterySafetyMode,
    SafeMode,
    ElectrolysisMode,
    ManeuverMode,
    SensorMode,
    TestMode,
    CommsMode,
)
from .opnav_flightmode import OpNavMode
from utils.constants import FMEnum
from utils.exceptions import UnknownFlightModeException

FLIGHT_MODE_DICT = {
    FMEnum.Boot.value: BootUpMode,
    FMEnum.Restart.value: RestartMode,
    FMEnum.Normal.value: NormalMode,
    FMEnum.LowBatterySafety.value: LowBatterySafetyMode,
    FMEnum.Safety.value: SafeMode,
    FMEnum.OpNav.value: OpNavMode,
    FMEnum.Electrolysis.value: ElectrolysisMode,
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
