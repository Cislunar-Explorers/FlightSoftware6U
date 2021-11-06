import utils.constants as consts


def test_env():
    assert "CISLUNAR_BASE_DIR" in consts.config
    assert "FOR_FLIGHT" in consts.config
    assert "LOG" in consts.config
