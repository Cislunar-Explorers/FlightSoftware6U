import pytest
from dotenv import dotenv_values

from utils.constants import config


@pytest.fixture
def test_env(mocker):
    test_config = dotenv_values(".env.test")
    mocker.patch.dict("utils.constants.config", values=test_config)


def test_normal_env():
    assert "DATABASE_URL" in config
    assert "FOR_FLIGHT" in config
    assert "LOG" in config
    assert config["FOR_FLIGHT"] == "1"


def test_test_env(test_env):
    assert "FOR_FLIGHT" in config
    assert config["FOR_FLIGHT"] == "0"
