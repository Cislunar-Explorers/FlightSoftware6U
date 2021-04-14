import utils.parameters
from utils.exceptions import CislunarException
from json import load
import os


def test_parameters():
    """Copied from main.py's init_parameters"""
    with open(os.fspath("../utils/parameters.json")) as f:
        json_parameter_dict = load(f)

    try:
        for parameter in utils.parameters.__dir__():
            if parameter[0] != '_':
                utils.parameters.__setattr__(parameter, json_parameter_dict[parameter])

    except:
        raise CislunarException(
            f'Attempted to set parameter {parameter}, which could not be found in parameters.json'
        )


if __name__ == '__main__':
    test_parameters()
