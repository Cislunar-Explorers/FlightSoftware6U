from random import randint
import drivers.power.power_structs as ps
from communications.commands import CommandHandler
from utils.constants import FMEnum, NormalCommandEnum, MAC
from communications.command_definitions import dict_from_eps_config, eps_config_from_dict


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

    command_bytes = ch.pack_command(COUNTER, FMEnum.Normal.value, NormalCommandEnum.GomConf1Set.value, **config_dict)
    mac, counter, mode, command_id, kwargs = ch.unpack_command(command_bytes)

    assert mac == MAC
    assert counter == COUNTER
    assert mode == FMEnum.Normal.value
    assert command_id == NormalCommandEnum.GomConf1Set.value

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


if __name__ == '__main__':
    test_config_command()
