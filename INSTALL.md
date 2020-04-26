# Install


# Raspian vs. Non-Raspian
Some modules can only be installed on Raspberry Pi while others can be installed on any system. In order to get around this, we use the extras_requires argument to setup in order to specify at the command line what version of our requirements we want to install.

Note: OpNav currently requires python3.4 or 3.5, so we may need to downgrade python versions in order to use OpenCV in the future.

Set up your virtual environment using virtualenv and activate it with:
```bash
virtualenv --python=python3.6 venv
source venv/activate/bin
```

Documentation for each option is noted in the definition of EXTRAS in the setup.py file. In order to install one of the options, start the virtual environment and run the following command:
```bash
pip install -e .[option]
```

# Laptop Installation
Warning: does not include installation of modules for peripherals due to the fact that they are not compatible with operating systems such as Linux, Windows, and MacOSX.
```bash
pip install -e .[dev]
```


# Installation for Raspberry Pi
Installs all modules.
```bash
pip install -e [rpi-dev]
```

# Set up Environment

Copy the file .env.example and create a file .env. Set the variable FOR_FLIGHT=FLIGHT should be configured for flight. Otherwise, this value will be set to false. If it's set to flight, then it will automatically restart itself on failure and clean up its use of storage space if it's running on the raspian operating system.