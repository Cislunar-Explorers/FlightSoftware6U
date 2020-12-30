# Cislunar Explorers Flight Software
![Mission Patch](mission_patch.png)

## About

### Mission
The Cislunar Explorers are a pair of CubeSats in a 6U configuration that are launching as a secondary payload onboard NASA's Artemis-1 launch. The mission objectives are to:
- Demonstrate a water electrolysis-based propulsion system
- Demonstrate an optical attitude and position determination system running on inexpensive COTS hardware
- Achieve Lunar orbit

### Flight Software
The software in this repository will run the satellite's flight computers, with the main loop is defined in the `run` method of `main.py`. Here, each step of the loop is defined by which flight mode the satellite is currently in.

## Set up Virtual Environment
```bash
python3 -m venv ./cislunar-venv
```

## Activate the Virtual Environment
```bash
. ./cislunar-venv/bin/activate
```

## Install dependencies with requirements files
We split up dependencies into two separate requirements files split up by whether they should only be installed on Raspberry Pi.

```
pip install -r requirements.txt
pip install -r requirements_pi.txt
```

## Install dependencies with setup.py

After starting the virtual environment, invoke the setup.py file using pip with one of the following options.
rpi: used for running on the Raspberry Pi
rpi-dev: used for active development on the Raspberry Pi
dev: used for development not on the Raspberry Pi

```
pip install -e .[rpi-dev]
```

This would install the rpi-dev option, which installs all of the potential dependencies.


## Adding Dependency Requirements to Build System

For now, we have both the requirements files as well as the setup.py file. We will eventually select only one based on what everyone finds to be the easiest to work with, but in the meantime we will keep both of them up to date.

When you need to install a dependency that will only be compatible with the Raspberry Pi, add its name to the file: requirements_pi.txt. In addition, add its name to the list variable in setup.py PI_INSTALL_REQUIRES.

When you need to add a dependency that does not depend exclusively on the Raspberry Pi, but will also work with Linux, Mac OSX, etc. you should add it to the requirments.txt file and the INSTALL_REQUIRES list variable in setup.py in the same way.
