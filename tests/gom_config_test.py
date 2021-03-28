import drivers.power.power_controller as pc
import drivers.power.power_structs as ps
from communications.commands import CommandHandler
from utils.constants import FMEnum, NormalCommandEnum, MAC
from communications.command_definitions import dict_from_eps_config, eps_config_from_dict


def test_config_command():
    config = ps.eps_config_t()
    config.ppt_mode = 2
    config.battheater_mode = 1
    config.battheater_low = 4
    config.battheater_high = 7
    config.output_normal_value = [0] * 8
    config.output_safe_value = [0] * 8
    config.output_initial_on_delay = [0] * 8
    config.output_initial_off_delay = [0] * 8
    config.vboost = [3500] * 3

    config_dict = dict_from_eps_config(config)

    COUNTER = 0

    ch = CommandHandler()

    command_bytes = ch.pack_command(COUNTER, FMEnum.Normal.value, NormalCommandEnum.GomConf1Set.value, **config_dict)
    mac, counter, mode, command_id, kwargs = ch.unpack_command(command_bytes)

    assert mac == MAC
    assert counter == COUNTER
    assert mode == FMEnum.Normal.value
    assert command_id == NormalCommandEnum.GomConf1Set.value
    assert config == eps_config_from_dict(**kwargs)
