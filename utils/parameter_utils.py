from json import load, dump
from utils.constants import PARAMETERS_JSON_PATH
from utils import parameters
from utils.exceptions import CislunarException
from typing import Dict, Any, List


def get_parameter_list(hard: bool = False, filename=PARAMETERS_JSON_PATH) -> List[str]:
    if hard:
        return list(load(open(filename)).keys())
    else:
        return [param_name for param_name in dir(parameters) if param_name[0] != '_']


def init_parameters(filename=PARAMETERS_JSON_PATH):
    with open(filename) as f:
        json_parameter_dict = load(f)

    for parameter in dir(parameters):
        try:
            if parameter[0] != '_':
                setattr(parameters, parameter, json_parameter_dict[parameter])
        except KeyError:
            raise CislunarException(
                f'Attempted to set parameter {parameter}, which could not be found in {PARAMETERS_JSON_PATH}'
            )


def set_parameter(name: str, value, hard_set: bool, filename=PARAMETERS_JSON_PATH):
    initial_value = getattr(parameters, name)
    setattr(parameters, name, value)

    # Hard sets new parameter value into JSON file
    if hard_set:
        with open(filename) as f:
            json_parameter_dict = load(f)
        json_parameter_dict[name] = value
        dump(json_parameter_dict, open(filename, 'w'), indent=0)

    return initial_value


def bulk_set_parameters(new_params: Dict[str, Any], hard_set: bool):
    for parameter_name, value in new_params.items():
        set_parameter(parameter_name, value, hard_set)
