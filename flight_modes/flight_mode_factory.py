from .flight_mode import (
    BootUpMode,
    RestartMode,
    NormalMode,
    LowBatteryMode,
    #SafeMode,
    ManeuverMode,
    #SensorMode,
    #TestMode,
    #CommsMode,
    OpNavManeuverMode
)
from .opnav_flightmode import OpNavMode
from utils.constants import FMEnum
from utils.exceptions import UnknownFlightModeException

FLIGHT_MODE_DICT = {
    FMEnum.Boot.value: BootUpMode,
    FMEnum.Restart.value: RestartMode,
    FMEnum.Normal.value: NormalMode,
    FMEnum.LowBattery.value: LowBatteryMode,
    FMEnum.Safety.value: SafeMode,
    FMEnum.OpNav.value: OpNavMode,
    FMEnum.Maneuver.value: ManeuverMode,
    FMEnum.SensorMode.value: SensorMode,
    FMEnum.TestMode.value: TestMode,
    FMEnum.CommsMode.value: CommsMode,
    FMEnum.OpNavManeuver.value: OpNavManeuverMode
}


def build_flight_mode(main, fm_id, **kwargs):
    try:
        cls = FLIGHT_MODE_DICT[fm_id]
        return cls(main)
    except KeyError:
        raise UnknownFlightModeException(fm_id)
