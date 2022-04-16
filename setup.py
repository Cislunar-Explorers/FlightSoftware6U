#!/usr/bin/env python

"""The setup script."""

from setuptools import setup, find_packages

AUTHOR = "Dr. Kyle Doyle PhD"
NAME = "CislunarExplorers"
DESCRIPTION = "Flight software for Cislunar-Explorers"

REQUIRES_PYTHON = ">=3.6"

VERSION = "0.1"

INCLUDES = ["*"]

INSTALL_REQUIRES = [
    "Adafruit-Blinka",
    "adafruit-circuitpython-busdevice",
    "adafruit-circuitpython-ds3231",
    "adafruit-circuitpython-fxas21002c",
    "adafruit-circuitpython-fxos8700",
    "ADS1115",
    "astropy",
    "bitstring",
    "numba",
    "numpy-quaternion",
    "numpy",
    "opencv-python-headless",
    "opencv-python",
    "pigpio",
    "psutil",
    "python-dotenv",
    "scipy",
    "sqlalchemy",
    "tqdm",
    "uptime" "parameterized",
]

PI_INSTALL_REQUIRES = [
    "adafruit-circuitpython-bno055",
    "board",
    "busio",
    "picamera",
    "vcgencmd",
]

DEV_REQUIRES = ["black", "flake8", "pip", "pre-commit"]

DOCS_REQUIRES = ["sphinx", "sphinx-rtd-theme", "watchdog"]

TEST_REQUIRES = ["matplotlib", "pandas", "pytest", "pytest-mock"]

EXTRAS = {"rpi": PI_INSTALL_REQUIRES, "dev": DEV_REQUIRES + TEST_REQUIRES}

setup(
    author=AUTHOR,
    python_requires=REQUIRES_PYTHON,
    desciption=DESCRIPTION,
    extras_require=EXTRAS,
    install_requires=INSTALL_REQUIRES,
    name=NAME,
    packages=find_packages(include=INCLUDES),
    version=VERSION,
)
