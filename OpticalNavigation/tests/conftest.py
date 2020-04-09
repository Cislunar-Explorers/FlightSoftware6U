import pytest

def pytest_addoption(parser):
    parser.addoption("--visual_analysis", action="store", default="False")

@pytest.fixture(scope='session')
def visual_analysis(request):
    name_value = request.config.option.visual_analysis
    if name_value is None:
        pytest.skip()
    return name_value
# def pytest_generate_tests(metafunc):
#     # This is called for every test. Only get/set command line arguments
#     # if the argument is specified in the list of test "fixturenames".
#     option_value = metafunc.config.option.name
#     if 'visual_analysis' in metafunc.fixturenames and option_value is not None:
#         metafunc.parametrize("visual_analysis", [option_value])