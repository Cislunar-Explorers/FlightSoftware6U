import logging
from logging.handlers import RotatingFileHandler
import sys

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(filename)s:%(lineno)s] %(message)s",
    handlers=[
        RotatingFileHandler("cislunar.log", maxBytes=4096, backupCount=10),
        logging.StreamHandler(sys.stdout),
    ],
)

log = logging.getLogger("CislunarExplorers")
# Manually change PIL logging level because still logs useless debug statements
logging.getLogger("PIL").setLevel(logging.WARNING)
