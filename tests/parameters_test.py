import utils.parameters
from utils.exceptions import CislunarException
from json import load
import os


def test_parameters():
    """Copied from main.py's init_parameters"""
    print(os.getcwd())
    if 'FlightSoftware/FlightSoftware/' not in os.getcwd():  # checks if running pytest
        filepath = 'utils/parameters.json'
    else:
        filepath = '../utils/parameters.json'

    with open(os.fspath(filepath)) as f:
        json_parameter_dict = load(f)

    parameters_missing = []
    for parameter in dir(utils.parameters):
        if parameter[0] != '_':
            try:
                setattr(utils.parameters, parameter, json_parameter_dict[parameter])
            except KeyError:
                parameters_missing.append(parameter)

    if len(parameters_missing) > 0:
        raise CislunarException(
            f'Attempted to set parameter(s) {parameters_missing}, which could not be found in parameters.json'
        )


if __name__ == '__main__':
    test_parameters()
