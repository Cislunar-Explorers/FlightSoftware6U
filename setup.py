from setuptools import setup, find_packages


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
