import drivers.power.power_structs as ps
import utils.constants as consts
from typing import List, cast


def dict_from_eps_config(config: ps.eps_config_t) -> dict:
    return {consts.PPT_MODE: config.ppt_mode,
            consts.BATTHEATERMODE: bool(config.battheater_mode),
            consts.BATTHEATERLOW: config.battheater_low,
            consts.BATTHEATERHIGH: config.battheater_high,
            consts.OUTPUT_NORMAL1: bool(config.output_normal_value[0]),
            consts.OUTPUT_NORMAL2: bool(config.output_normal_value[1]),
            consts.OUTPUT_NORMAL3: bool(config.output_normal_value[2]),
            consts.OUTPUT_NORMAL4: bool(config.output_normal_value[3]),
            consts.OUTPUT_NORMAL5: bool(config.output_normal_value[4]),
            consts.OUTPUT_NORMAL6: bool(config.output_normal_value[5]),
            consts.OUTPUT_NORMAL7: bool(config.output_normal_value[6]),
            consts.OUTPUT_NORMAL8: bool(config.output_normal_value[7]),
            consts.OUTPUT_SAFE1: bool(config.output_safe_value[0]),
            consts.OUTPUT_SAFE2: bool(config.output_safe_value[1]),
            consts.OUTPUT_SAFE3: bool(config.output_safe_value[2]),
            consts.OUTPUT_SAFE4: bool(config.output_safe_value[3]),
            consts.OUTPUT_SAFE5: bool(config.output_safe_value[4]),
            consts.OUTPUT_SAFE6: bool(config.output_safe_value[5]),
            consts.OUTPUT_SAFE7: bool(config.output_safe_value[6]),
            consts.OUTPUT_SAFE8: bool(config.output_safe_value[7]),
            consts.OUTPUT_ON_DELAY: config.output_initial_on_delay[0],
            consts.OUTPUT_OFF_DELAY: config.output_initial_off_delay[0],
            consts.VBOOST1: config.vboost[0],
            consts.VBOOST2: config.vboost[1],
            consts.VBOOST3: config.vboost[2],
            }


def eps_config_from_dict(**kwargs) -> ps.eps_config_t:
    ppt_mode = kwargs.get(consts.PPT_MODE)
    # BATTHEATERMODE is transmitted as a bool, then cast to 0/1
    heater_mode = int(kwargs[consts.BATTHEATERMODE])
    heater_low = kwargs.get(consts.BATTHEATERLOW)
    heater_high = kwargs.get(consts.BATTHEATERHIGH)
    normal_output: List[bool] = cast('List[bool]', [kwargs.get(consts.OUTPUT_NORMAL1),
                                                    kwargs.get(
                                                        consts.OUTPUT_NORMAL2),
                                                    kwargs.get(
                                                        consts.OUTPUT_NORMAL3),
                                                    kwargs.get(
                                                        consts.OUTPUT_NORMAL4),
                                                    kwargs.get(
                                                        consts.OUTPUT_NORMAL5),
                                                    kwargs.get(
                                                        consts.OUTPUT_NORMAL6),
                                                    kwargs.get(
                                                        consts.OUTPUT_NORMAL7),
                                                    kwargs.get(consts.OUTPUT_NORMAL8)])

    # transmitted as bools, convert to ints
    normal_output_int = list(map(int, normal_output))

    safe_output: List[bool] = cast('List[bool]', [kwargs.get(consts.OUTPUT_SAFE1),
                                                  kwargs.get(
                                                      consts.OUTPUT_SAFE2),
                                                  kwargs.get(
                                                      consts.OUTPUT_SAFE3),
                                                  kwargs.get(
                                                      consts.OUTPUT_SAFE4),
                                                  kwargs.get(
                                                      consts.OUTPUT_SAFE5),
                                                  kwargs.get(
                                                      consts.OUTPUT_SAFE6),
                                                  kwargs.get(
                                                      consts.OUTPUT_SAFE7),
                                                  kwargs.get(consts.OUTPUT_SAFE8)])
    # transmitted as bools, convert to ints
    safe_output_int: List[int] = list(map(int, safe_output))

    # this means that all outputs have the same on/off delay
    initial_on_delay = [kwargs.get(consts.OUTPUT_ON_DELAY)] * 8
    initial_off_delay = [kwargs.get(consts.OUTPUT_OFF_DELAY)] * 8

    vboost = [kwargs.get(consts.VBOOST1), kwargs.get(
        consts.VBOOST2), kwargs.get(consts.VBOOST3)]

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


def eps_config2_from_dict(config_dict: dict) -> ps.eps_config2_t:
    gom_conf2 = ps.eps_config2_t()

    max_voltage = config_dict.get(consts.MAX_VOLTAGE)
    normal_voltage = config_dict.get(consts.NORM_VOLTAGE)
    safe_voltage = config_dict.get(consts.SAFE_VOLTAGE)
    crit_voltage = config_dict.get(consts.CRIT_VOLTAGE)

    gom_conf2.batt_maxvoltage = max_voltage
    gom_conf2.batt_normalvoltage = normal_voltage
    gom_conf2.batt_safevoltage = safe_voltage
    gom_conf2.batt_criticalvoltage = crit_voltage

    return gom_conf2


def dict_from_eps_config2(conf2: ps.eps_config2_t) -> dict:
    return {consts.MAX_VOLTAGE: conf2.batt_maxvoltage,
            consts.NORM_VOLTAGE: conf2.batt_normalvoltage,
            consts.SAFE_VOLTAGE: conf2.batt_safevoltage,
            consts.CRIT_VOLTAGE: conf2.batt_criticalvoltage}
