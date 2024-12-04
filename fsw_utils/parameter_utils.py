from json import load, dump
from fsw_utils.constants import PARAMETERS_JSON_PATH
from fsw_utils import parameters
from fsw_utils.exceptions import CislunarException
from typing import Dict, Any, List, Union
import os


def get_parameter_list(hard: bool = False, filename=PARAMETERS_JSON_PATH) -> List[str]:
    if hard:
        return list(load(open(filename)).keys())
    else:
        return [param_name for param_name in dir(parameters) if param_name[0] != "_"]


def get_parameter_from_name(param_name: str) -> Union[str, int, float]:
    return getattr(parameters, param_name)


def init_parameters(filename=PARAMETERS_JSON_PATH):
    if not os.path.exists(filename):
        # if the parameters.json file doesn't exist, write it
        # get all parameter names
        param_name_list = get_parameter_list(hard=False)
        param_list = [get_parameter_from_name(name) for name in param_name_list]
        param_dict = dict(zip(param_name_list, param_list))

        with open(filename, "w") as f:
            dump(param_dict, f)

    with open(filename) as f:
        json_parameter_dict = load(f)

    for parameter in dir(parameters):
        try:
            if parameter[0] != "_":
                setattr(parameters, parameter, json_parameter_dict[parameter])
        except KeyError:
            raise CislunarException(
                f"Attempted to set parameter {parameter}, which could not be found in {PARAMETERS_JSON_PATH}"
            )


def set_parameter(name: str, value, hard_set: bool, filename=PARAMETERS_JSON_PATH):
    initial_value = getattr(parameters, name)
    setattr(parameters, name, value)

    # Hard sets new parameter value into JSON file
    if hard_set:
        with open(filename) as f:
            json_parameter_dict = load(f)
        json_parameter_dict[name] = value
        dump(json_parameter_dict, open(filename, "w"), indent=0)

    return initial_value


def bulk_set_parameters(new_params: Dict[str, Any], hard_set: bool):
    for parameter_name, value in new_params.items():
        set_parameter(parameter_name, value, hard_set)
