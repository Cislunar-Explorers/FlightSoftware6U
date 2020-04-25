from setuptools import setup, find_packages

# Directions:
# Some modules can only be installed on Raspberry Pi while others can be installed on any system
# In order to get around this, we use the extras_requires argument to setup in order to specify
# at the command line what version of our requirements we want to install.
# Documentation for each option is noted in the definition of EXTRAS below.
# In order to install one of the options, start the virtual environment and run the following command:
# pip install -e .[option]

AUTHOR = "Dr. Kyle Doyle PhD"
NAME = "CislunarExplorers"
DESCRIPTION = "Flight software for Cislunar-Explorers"

REQUIRES_PYTHON = ">=3.6"

VERSION = "0.1"

INSTALL_REQUIRES = [
    "requests",
    "numpy",
    "opencv-python",
    "SQLAlchemy",
]

PI_INSTALL_REQUIRES = [
    "ADS1115",
    "picamera",
    "board",
    "busio",
    "adafruit-circuitpython-bno055",
]

DEV_REQUIRES = [
    "black",
    "flake8",
]

TESTS_REQUIRES = [
    "pytest",
    "pandas",
]


EXTRAS = {
    "rpi": INSTALL_REQUIRES
    + PI_INSTALL_REQUIRES,  # Install everything needed for running on RPi
    "rpi-dev": INSTALL_REQUIRES
    + PI_INSTALL_REQUIRES
    + DEV_REQUIRES
    + TESTS_REQUIRES,  # Install for active development on RPi
    "dev": INSTALL_REQUIRES
    + DEV_REQUIRES
    + TESTS_REQUIRES,  # Install for development, not on RPi
}

setup(
    author=AUTHOR,
    desciption=DESCRIPTION,
    extras_requires=EXTRAS,
    install_requires=INSTALL_REQUIRES,
    name=NAME,
    packages=find_packages(),
    python_requires=REQUIRES_PYTHON,
    version=VERSION,
)
