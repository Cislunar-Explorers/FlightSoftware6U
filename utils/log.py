import logging
from typing import Callable
from logging.handlers import RotatingFileHandler
import sys

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(filename)s:%(lineno)s] %(message)s",
    handlers=[
        RotatingFileHandler("cislunar.log", maxBytes=4096, backupCount=10),
        logging.StreamHandler(sys.stdout),
    ]
)

log = logging.getLogger("CislunarExplorers")


def get_log():
    return log


def log_error(e: Exception, function: Callable = log.error, **kwargs):
    """Logs an exception, e, using any function. function is usually either log.error, log.warning, or print. The
    kwarg exc_info=1 is useful for logging the entire traceback for log.error """
    function(e, **kwargs)
