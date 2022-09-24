from utils.log import *
import opnav.core.sense


def test_camera():
    logging.debug("Testing camera")
    opnav.core.sense.select_camera(0)
    file_diff_time = opnav.core.sense.record_video("test", 30, 5, 1000)
    logging.debug(file_diff_time)
