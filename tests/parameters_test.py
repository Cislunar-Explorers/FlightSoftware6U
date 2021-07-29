import os
from utils import parameter_utils, parameters
import json
import unittest


class ParametersTestCase(unittest.TestCase):
    def setUp(self) -> None:
        print(os.getcwd())
        if 'FlightSoftware/FlightSoftware/' not in os.getcwd():  # checks if running pytest
            self.filepath = 'utils/parameters.json'
        else:
            self.filepath = '../utils/parameters.json'

    def test_parameter_consistency(self):
        """Tests whether the parameters defined in parameters.json are consistent with those defined in parameters.py"""

        py_list = parameter_utils.get_parameter_list()
        json_list = parameter_utils.get_parameter_list(hard=True, filename=self.filepath)
        inconsistencies = set(py_list).symmetric_difference(json_list)
        if not inconsistencies:
            raise ValueError(f"These parameter names are not consistent: {inconsistencies}")

    def test_parameters_init(self):
        parameter_utils.init_parameters(filename=self.filepath)

    def test_parameter_set(self):
        param_name = 'ACS_SPIKE_DURATION'
        new_parm_value = 20
        parameter_utils.set_parameter(param_name, new_parm_value, True, filename=self.filepath)
        self.assertEqual(new_parm_value, parameters.ACS_SPIKE_DURATION)
        self.assertEqual(new_parm_value, json.load(open(self.filepath))[param_name])


if __name__ == '__main__':
    unittest.main()
