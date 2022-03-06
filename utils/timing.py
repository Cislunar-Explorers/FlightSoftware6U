from time import time, sleep


def wait(seconds: float):
    sleep(max([0, seconds]))


def get_time() -> float:
    return time()


def wait_until(timestamp: float):
    wait(timestamp - get_time())


def mock_wait(seconds: float):
    pass


def mock_get_time() -> float:
    raise NotImplementedError
