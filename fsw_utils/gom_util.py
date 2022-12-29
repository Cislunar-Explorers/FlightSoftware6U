import drivers.power.power_structs as ps
from fsw_utils.constants import GomConfKwargs
from typing import Dict, List, cast


def dict_from_eps_config(config: ps.eps_config_t) -> dict:
    return {
        GomConfKwargs.PPT_MODE: config.ppt_mode,
        GomConfKwargs.BATTHEATERMODE: bool(config.battheater_mode),
        GomConfKwargs.BATTHEATERLOW: config.battheater_low,
        GomConfKwargs.BATTHEATERHIGH: config.battheater_high,
        GomConfKwargs.OUTPUT_NORMAL1: bool(config.output_normal_value[0]),
        GomConfKwargs.OUTPUT_NORMAL2: bool(config.output_normal_value[1]),
        GomConfKwargs.OUTPUT_NORMAL3: bool(config.output_normal_value[2]),
        GomConfKwargs.OUTPUT_NORMAL4: bool(config.output_normal_value[3]),
        GomConfKwargs.OUTPUT_NORMAL5: bool(config.output_normal_value[4]),
        GomConfKwargs.OUTPUT_NORMAL6: bool(config.output_normal_value[5]),
        GomConfKwargs.OUTPUT_NORMAL7: bool(config.output_normal_value[6]),
        GomConfKwargs.OUTPUT_NORMAL8: bool(config.output_normal_value[7]),
        GomConfKwargs.OUTPUT_SAFE1: bool(config.output_safe_value[0]),
        GomConfKwargs.OUTPUT_SAFE2: bool(config.output_safe_value[1]),
        GomConfKwargs.OUTPUT_SAFE3: bool(config.output_safe_value[2]),
        GomConfKwargs.OUTPUT_SAFE4: bool(config.output_safe_value[3]),
        GomConfKwargs.OUTPUT_SAFE5: bool(config.output_safe_value[4]),
        GomConfKwargs.OUTPUT_SAFE6: bool(config.output_safe_value[5]),
        GomConfKwargs.OUTPUT_SAFE7: bool(config.output_safe_value[6]),
        GomConfKwargs.OUTPUT_SAFE8: bool(config.output_safe_value[7]),
        GomConfKwargs.OUTPUT_ON_DELAY: config.output_initial_on_delay[0],
        GomConfKwargs.OUTPUT_OFF_DELAY: config.output_initial_off_delay[0],
        GomConfKwargs.VBOOST1: config.vboost[0],
        GomConfKwargs.VBOOST2: config.vboost[1],
        GomConfKwargs.VBOOST3: config.vboost[2],
    }


def eps_config_from_dict(config_dict: Dict[str, int]) -> ps.eps_config_t:
    ppt_mode = config_dict[GomConfKwargs.PPT_MODE]
    # BATTHEATERMODE is transmitted as a bool, then cast to 0/1
    heater_mode = int(config_dict[GomConfKwargs.BATTHEATERMODE])
    heater_low = config_dict[GomConfKwargs.BATTHEATERLOW]
    heater_high = config_dict[GomConfKwargs.BATTHEATERHIGH]
    normal_output: List[bool] = cast(
        "List[bool]",
        [
            config_dict[GomConfKwargs.OUTPUT_NORMAL1],
            config_dict[GomConfKwargs.OUTPUT_NORMAL2],
            config_dict[GomConfKwargs.OUTPUT_NORMAL3],
            config_dict[GomConfKwargs.OUTPUT_NORMAL4],
            config_dict[GomConfKwargs.OUTPUT_NORMAL5],
            config_dict[GomConfKwargs.OUTPUT_NORMAL6],
            config_dict[GomConfKwargs.OUTPUT_NORMAL7],
            config_dict[GomConfKwargs.OUTPUT_NORMAL8],
        ],
    )

    # transmitted as bools, convert to ints
    normal_output_int = list(map(int, normal_output))

    safe_output: List[bool] = cast(
        "List[bool]",
        [
            config_dict[GomConfKwargs.OUTPUT_SAFE1],
            config_dict[GomConfKwargs.OUTPUT_SAFE2],
            config_dict[GomConfKwargs.OUTPUT_SAFE3],
            config_dict[GomConfKwargs.OUTPUT_SAFE4],
            config_dict[GomConfKwargs.OUTPUT_SAFE5],
            config_dict[GomConfKwargs.OUTPUT_SAFE6],
            config_dict[GomConfKwargs.OUTPUT_SAFE7],
            config_dict[GomConfKwargs.OUTPUT_SAFE8],
        ],
    )
    # transmitted as bools, convert to ints
    safe_output_int: List[int] = list(map(int, safe_output))

    # this means that all outputs have the same on/off delay
    initial_on_delay = [config_dict[GomConfKwargs.OUTPUT_ON_DELAY]] * 8
    initial_off_delay = [config_dict[GomConfKwargs.OUTPUT_OFF_DELAY]] * 8

    vboost = [
        config_dict[GomConfKwargs.VBOOST1],
        config_dict[GomConfKwargs.VBOOST2],
        config_dict[GomConfKwargs.VBOOST3],
    ]

    new_config = ps.eps_config_t()
    new_config.ppt_mode = ppt_mode
    new_config.battheater_mode = heater_mode
    new_config.battheater_low = heater_low
    new_config.battheater_high = heater_high

    new_config.output_normal_value[0] = normal_output_int[0]
    new_config.output_normal_value[1] = normal_output_int[1]
    new_config.output_normal_value[2] = normal_output_int[2]
    new_config.output_normal_value[3] = normal_output_int[3]
    new_config.output_normal_value[4] = normal_output_int[4]
    new_config.output_normal_value[5] = normal_output_int[5]
    new_config.output_normal_value[6] = normal_output_int[6]
    new_config.output_normal_value[7] = normal_output_int[7]

    new_config.output_safe_value[0] = safe_output_int[0]
    new_config.output_safe_value[1] = safe_output_int[1]
    new_config.output_safe_value[2] = safe_output_int[2]
    new_config.output_safe_value[3] = safe_output_int[3]
    new_config.output_safe_value[4] = safe_output_int[4]
    new_config.output_safe_value[5] = safe_output_int[5]
    new_config.output_safe_value[6] = safe_output_int[6]
    new_config.output_safe_value[7] = safe_output_int[7]

    new_config.output_initial_on_delay[0] = initial_on_delay[0]
    new_config.output_initial_on_delay[1] = initial_on_delay[1]
    new_config.output_initial_on_delay[2] = initial_on_delay[2]
    new_config.output_initial_on_delay[3] = initial_on_delay[3]
    new_config.output_initial_on_delay[4] = initial_on_delay[4]
    new_config.output_initial_on_delay[5] = initial_on_delay[5]
    new_config.output_initial_on_delay[6] = initial_on_delay[6]
    new_config.output_initial_on_delay[7] = initial_on_delay[7]

    new_config.output_initial_off_delay[0] = initial_off_delay[0]
    new_config.output_initial_off_delay[1] = initial_off_delay[1]
    new_config.output_initial_off_delay[2] = initial_off_delay[2]
    new_config.output_initial_off_delay[3] = initial_off_delay[3]
    new_config.output_initial_off_delay[4] = initial_off_delay[4]
    new_config.output_initial_off_delay[5] = initial_off_delay[5]
    new_config.output_initial_off_delay[6] = initial_off_delay[6]
    new_config.output_initial_off_delay[7] = initial_off_delay[7]

    new_config.vboost[0] = vboost[0]
    new_config.vboost[1] = vboost[1]
    new_config.vboost[2] = vboost[2]

    return new_config


def eps_config2_from_dict(config_dict: Dict[str, int]) -> ps.eps_config2_t:
    gom_conf2 = ps.eps_config2_t()

    max_voltage = config_dict[GomConfKwargs.MAX_VOLTAGE]
    normal_voltage = config_dict[GomConfKwargs.NORM_VOLTAGE]
    safe_voltage = config_dict[GomConfKwargs.SAFE_VOLTAGE]
    crit_voltage = config_dict[GomConfKwargs.CRIT_VOLTAGE]

    gom_conf2.batt_maxvoltage = max_voltage
    gom_conf2.batt_normalvoltage = normal_voltage
    gom_conf2.batt_safevoltage = safe_voltage
    gom_conf2.batt_criticalvoltage = crit_voltage

    return gom_conf2


def dict_from_eps_config2(conf2) -> Dict[str, int]:
    return {
        GomConfKwargs.MAX_VOLTAGE: conf2.batt_maxvoltage,
        GomConfKwargs.NORM_VOLTAGE: conf2.batt_normalvoltage,
        GomConfKwargs.SAFE_VOLTAGE: conf2.batt_safevoltage,
        GomConfKwargs.CRIT_VOLTAGE: conf2.batt_criticalvoltage,
    }
