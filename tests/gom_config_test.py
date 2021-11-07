from random import randint
import drivers.power.power_structs as ps
from communications.command_handler import CommandHandler
from utils.constants import CommandEnum
from utils.gom_util import dict_from_eps_config, eps_config_from_dict, dict_from_eps_config2, \
    eps_config2_from_dict


def test_config_command():
    config = ps.eps_config_t()
    config.ppt_mode = randint(1, 2)
    config.battheater_mode = randint(0, 1)
    config.battheater_low = randint(0, 5)
    config.battheater_high = randint(6, 20)
    config.output_normal_value[0] = randint(0, 1)
    config.output_normal_value[1] = randint(0, 1)
    config.output_normal_value[2] = randint(0, 1)
    config.output_normal_value[3] = randint(0, 1)
    config.output_normal_value[4] = randint(0, 1)
    config.output_normal_value[5] = randint(0, 1)
    config.output_normal_value[6] = randint(0, 1)
    config.output_normal_value[7] = randint(0, 1)

    config.output_safe_value[0] = randint(0, 1)
    config.output_safe_value[1] = randint(0, 1)
    config.output_safe_value[2] = randint(0, 1)
    config.output_safe_value[3] = randint(0, 1)
    config.output_safe_value[4] = randint(0, 1)
    config.output_safe_value[5] = randint(0, 1)
    config.output_safe_value[6] = randint(0, 1)
    config.output_safe_value[7] = randint(0, 1)

    initial_on = randint(0, 65535)
    config.output_initial_on_delay[0] = initial_on
    config.output_initial_on_delay[1] = initial_on
    config.output_initial_on_delay[2] = initial_on
    config.output_initial_on_delay[3] = initial_on
    config.output_initial_on_delay[4] = initial_on
    config.output_initial_on_delay[5] = initial_on
    config.output_initial_on_delay[6] = initial_on
    config.output_initial_on_delay[7] = initial_on

    initial_off = randint(0, 65535)
    config.output_initial_off_delay[0] = initial_off
    config.output_initial_off_delay[1] = initial_off
    config.output_initial_off_delay[2] = initial_off
    config.output_initial_off_delay[3] = initial_off
    config.output_initial_off_delay[4] = initial_off
    config.output_initial_off_delay[5] = initial_off
    config.output_initial_off_delay[6] = initial_off
    config.output_initial_off_delay[7] = initial_off

    config.vboost[0] = randint(1000, 4000)
    config.vboost[1] = randint(1000, 4000)
    config.vboost[2] = randint(1000, 4000)

    config_dict = dict_from_eps_config(config)

    COUNTER = 0

    ch = CommandHandler()

    command_bytes = ch.pack_link(
        True, COUNTER, CommandEnum.GomConf1Set.value, config_dict)
    command, kwargs = ch.unpack_link(command_bytes)

    assert command.id == CommandEnum.GomConf1Set.value

    unpacked_config = eps_config_from_dict(**kwargs)

    # ps.displayConfig(config)
    # ps.displayConfig(unpacked_config)
    assert config.ppt_mode == unpacked_config.ppt_mode
    assert config.battheater_mode == unpacked_config.battheater_mode
    assert config.battheater_low == unpacked_config.battheater_low
    assert config.battheater_high == unpacked_config.battheater_high

    assert config.output_normal_value[0] == unpacked_config.output_normal_value[0]
    assert config.output_normal_value[1] == unpacked_config.output_normal_value[1]
    assert config.output_normal_value[2] == unpacked_config.output_normal_value[2]
    assert config.output_normal_value[3] == unpacked_config.output_normal_value[3]
    assert config.output_normal_value[4] == unpacked_config.output_normal_value[4]
    assert config.output_normal_value[5] == unpacked_config.output_normal_value[5]
    assert config.output_normal_value[6] == unpacked_config.output_normal_value[6]
    assert config.output_normal_value[7] == unpacked_config.output_normal_value[7]

    assert config.output_safe_value[0] == unpacked_config.output_safe_value[0]
    assert config.output_safe_value[1] == unpacked_config.output_safe_value[1]
    assert config.output_safe_value[2] == unpacked_config.output_safe_value[2]
    assert config.output_safe_value[3] == unpacked_config.output_safe_value[3]
    assert config.output_safe_value[4] == unpacked_config.output_safe_value[4]
    assert config.output_safe_value[5] == unpacked_config.output_safe_value[5]
    assert config.output_safe_value[6] == unpacked_config.output_safe_value[6]
    assert config.output_safe_value[7] == unpacked_config.output_safe_value[7]

    assert config.output_initial_on_delay[0] == unpacked_config.output_initial_on_delay[0]
    assert config.output_initial_off_delay[0] == unpacked_config.output_initial_off_delay[0]

    assert config.vboost[0] == unpacked_config.vboost[0]
    assert config.vboost[1] == unpacked_config.vboost[1]
    assert config.vboost[2] == unpacked_config.vboost[2]


def test_config2_command():
    config2 = ps.eps_config2_t()
    config2.batt_maxvoltage = 8300
    config2.batt_normalvoltage = 7600
    config2.batt_safevoltage = 7200
    config2.batt_criticalvoltage = 6800

    config2_dict = dict_from_eps_config2(config2)

    COUNTER = 0

    ch = CommandHandler()

    command_bytes = ch.pack_link(
        True, COUNTER, CommandEnum.GomConf2Set.value, config2_dict)
    command, kwargs = ch.unpack_link(command_bytes)

    assert command.id == CommandEnum.GomConf2Set.value

    unpacked_config2 = eps_config2_from_dict(kwargs)

    assert config2._fields_ == unpacked_config2._fields_
    # ps.displayConfig2(config2)
    # ps.displayConfig2(unpacked_config2)

    assert config2.batt_maxvoltage == unpacked_config2.batt_maxvoltage
    assert config2.batt_normalvoltage == unpacked_config2.batt_normalvoltage
    assert config2.batt_safevoltage == unpacked_config2.batt_safevoltage
    assert config2.batt_criticalvoltage == unpacked_config2.batt_criticalvoltage


def prep_config():
    config = ps.eps_config_t()
    config.ppt_mode = 1
    config.battheater_mode = 1
    config.battheater_low = 0
    config.battheater_high = 1
    config.output_normal_value[0] = 0
    config.output_normal_value[1] = 0
    config.output_normal_value[2] = 0
    config.output_normal_value[3] = 0
    config.output_normal_value[4] = 0
    config.output_normal_value[5] = 0
    config.output_normal_value[6] = 0
    config.output_normal_value[7] = 0

    config.output_safe_value[0] = 0
    config.output_safe_value[1] = 0
    config.output_safe_value[2] = 0
    config.output_safe_value[3] = 0
    config.output_safe_value[4] = 0
    config.output_safe_value[5] = 0
    config.output_safe_value[6] = 0
    config.output_safe_value[7] = 0

    initial_on = 0
    config.output_initial_on_delay[0] = initial_on
    config.output_initial_on_delay[1] = initial_on
    config.output_initial_on_delay[2] = initial_on
    config.output_initial_on_delay[3] = initial_on
    config.output_initial_on_delay[4] = initial_on
    config.output_initial_on_delay[5] = initial_on
    config.output_initial_on_delay[6] = initial_on
    config.output_initial_on_delay[7] = initial_on

    initial_off = 0
    config.output_initial_off_delay[0] = initial_off
    config.output_initial_off_delay[1] = initial_off
    config.output_initial_off_delay[2] = initial_off
    config.output_initial_off_delay[3] = initial_off
    config.output_initial_off_delay[4] = initial_off
    config.output_initial_off_delay[5] = initial_off
    config.output_initial_off_delay[6] = initial_off
    config.output_initial_off_delay[7] = initial_off

    config.vboost[0] = 2410
    config.vboost[1] = 2410
    config.vboost[2] = 2410

    config_dict = dict_from_eps_config(config)
    return config_dict


if __name__ == '__main__':
    test_config_command()
    test_config2_command()
    print(prep_config())
