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

flight_mode_dict = {
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
        cls = flight_mode_dict[fm_id]
        return cls(main)
    except KeyError:
        raise UnknownFlightModeException(fm_id)
