# Contributing

## Types of Contributions

### Report Bugs

Report bugs at
[https://github.com/Cislunar-Explorers/FlightSoftware/issues](https://github.com/Cislunar-Explorers/FlightSoftware/issues).

If you are reporting a bug, please include:

- Any details about your local setup that might be helpful in troubleshooting.
- Detailed steps to reproduce the bug.

### Fix Bugs

Look through the GitHub issues for bugs. Anything tagged with "bug" and "help
wanted" is open to whoever wants to implement it.

### Implement Features

Look through the GitHub issues for features. Anything tagged with "enhancement"
and "help wanted" is open to whoever wants to implement it.

### Write Documentation

Cislunar could always use more documentation, whether as part of the official
Cislunar docs, in docstrings, or even on the web in blog posts, articles, and
such.

### Submit Feedback

The best way to send feedback is to file an issue at
[https://github.com/Cislunar-Explorers/FlightSoftware/issues](https://github.com/Cislunar-Explorers/FlightSoftware/issues).

If you are proposing a feature:

- Explain in detail how it would work.
- Keep the scope as narrow as possible, to make it easier to implement.
- Remember that this is a volunteer-driven project, and that contributions are
  welcome :)

## Get Started

Ready to contribute? Here's how to set up `FlightSoftware` for local
development.

1. Fork or clone the repo.

2. Setup the environment. The project comes with 2 optional dependency lists:
   `rpi` (for running on the Raspberry Pi) and `dev` (for code development).

   ```bash
   python -m venv venv
   source ./venv/bin/activate
   python -m pip install --upgrade pip
   pip install -e ."[rpi, dev]" # or just [dev]
   pip install -e ./OpticalNavigation
   pre-commit install
   ```

3. Create a branch for local development.

   ```bash
   git checkout -b name-of-your-bugfix-or-feature
   ```

4. Commit your changes and push your branch to GitHub::

   ```bash
   git add .
   git commit -m "Your detailed description of your changes."
   git push origin name-of-your-bugfix-or-feature
   ```

5. Submit a pull request through the GitHub website.

### Pull Request Guidelines

Before you submit a pull request, check that it meets these guidelines:

1. The pull request should include tests.
2. If the pull request adds functionality, the docs should be updated. Put your
   new functionality into a function with a docstring.

## Adding Dependency Requirements to Build System

For now, we have both the requirements files as well as the [setup.py](setup.py)
file. We will eventually select only one based on what everyone finds to be the
easiest to work with, but in the meantime we will keep both of them up to date.

When you need to install a dependency that:

- is **only** compatible with the Raspberry Pi: add its name to the file:
  [requirements_pi.txt](requirements_pi.txt) and add its name to the list
  variable `PI_INSTALL_REQUIRES` in [setup.py](setup.py).
- does **not** depend exclusively on the Raspberry Pi, but will also work with
  Linux, Mac OSX, etc., you should add it to the
  [requirements.txt](requirements.txt) file and the `INSTALL_REQUIRES` list
  variable in [setup.py](setup.py).

# Raspian vs. Non-Raspian

Some modules can only be installed on Raspberry Pi while others can be installed
on any system. In order to get around this, we use the extras_requires argument
to setup in order to specify at the command line what version of our
requirements we want to install.

Note: OpNav currently requires python3.4 or 3.5, so we may need to downgrade
python versions in order to use OpenCV in the future.

Set up your virtual environment using virtualenv and activate it with:

```bash
virtualenv --python=python3.6 venv
source venv/activate/bin
```

Documentation for each option is noted in the definition of EXTRAS in the
setup.py file. In order to install one of the options, start the virtual
environment and run the following command:

```bash
pip install -e .[option]
```

# Laptop Installation

Warning: does not include installation of modules for peripherals due to the
fact that they are not compatible with operating systems such as Linux, Windows,
and MacOSX.

```bash
pip install -e .[dev]
```

# Installation for Raspberry Pi

Installs all modules.

```bash
pip install -e [rpi-dev]
```

# Set up Environment

Copy the file .env.example and create a file .env. Set the variable
FOR_FLIGHT=FLIGHT should be configured for flight. Otherwise, this value will be
set to false. If it's set to flight, then it will automatically restart itself
on failure and clean up its use of storage space if it's running on the raspian
operating system.

```

```
