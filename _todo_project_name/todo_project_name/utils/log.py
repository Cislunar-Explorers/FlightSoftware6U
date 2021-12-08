import logging


def set_up_logging() -> None:
    format = "(%(threadName)-10s) %(asctime)s: %(message)s"
    logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
    # logging.getLogger().setLevel(logging.DEBUG)
