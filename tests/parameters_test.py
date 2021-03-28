from utils.constants import PARAMETERS_JSON_PATH
import utils.parameters
from utils.exceptions import CislunarException
from json import load


def test_parameters():
    """Copied from main.py's init_parameters"""
    with open(PARAMETERS_JSON_PATH) as f:
        json_parameter_dict = load(f)

    try:
        for parameter in utils.parameters.__dir__():
            if parameter[0] != '_':
                utils.parameters.__setattr__(parameter, json_parameter_dict[parameter])
    except:
        raise CislunarException(
            f'Attempted to set parameter {parameter}, which could not be found in parameters.json'
        )
