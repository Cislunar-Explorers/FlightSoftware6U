# Gomspace Battery Drivers
Software drivers that control the Gomspace NanoPower P31u from the onboard Raspberry Pi

## Requirements

The Python libraries required for these drivers to run on the pi are: 
 - `RPi.GPIO`
 - `pigpio`
 
Additionally, a pigpio daemon (`pigpiod`) must be running and active on the pi before running any functions (Talk to Dr. Muhlberger about setting this up).
The status of `pigpiod` can be found by running `systemctl status pigpiod`. If it is not running run `sudo pigpiod` 

## Documentation
Documentation for the low-level drivers defined in `power_structs.py` and `power_controller.py` can be viewed at https://cornell.app.box.com/file/225179481737 and https://cornell.app.box.com/file/225179502007 respectively.

## Functions

The drivers defined in `power_controller.py` and `power_structs.py` and their statuses are shown here:

| Function            | Tested | Status                                               | Test Date | Short Description                                                     |
|---------------------|:------:|:----------------------------------------------------:|-----------|-----------------------------------------------------------------------|
| `__init__`          | x      | Working                                              | SP20      | Creates a `Power` object to control the P31u                          |
| `displayAll`        | x      | Working                                              | SP20      | Prints housekeeping (hk), config, and config2 data                    |
| `write`             | x      | Working                                              | SP20      | Writes byte list to a command register                                |
| `read`              | x      | Working                                              | SP20      | Reads byte list from the P31u                                         |
| `ping`              |        | Unknown                                              |           | Pings value                                                           |
| `reboot`            | x      | Working                                              | SP20      | Reboots the P31u without cycling permanent outputs                    |
| `get_hk_1`          | x      | Working                                              | SP20      | Returns hk data                                                       |
| `get_hk_2`          | x      | Working                                              | 2020-10-21| Returns hk data                                                       |
| `get_hk_2_vi`       | x      | Working                                              | 2020-10-21| Returns electrical portion of `get_hk_2`                              |
| `get_hk_out`        | x      | Working                                              | 2020-10-21|                                                                       |
| `get_hk_wdt`        | x      | Working                                              | 2020-10-21|                                                                       |
| `get_hk_2_basic`    | x      | Working                                              | 2020-10-21|                                                                       |
| `set_output`        | x      | Working                                              | SP20      | Sets voltage output channels with bit mask                            |
| `set_single_output` | x      | Working                                              | SP20      | Sets a single switchable output on or off                             |
| `set_pv_volt`       |        | Unknown                                              |           | Sets voltage of photovoltaic inputs                                   |
| `set_pv_auto`       |        | Unknown                                              |           | Sets solar cell tracking mode                                         |
| `set_heater`        |        | Unknown                                              |           | Sets heater on/off, returns heater mode                               |
| `get_heater`        |        | Unknown                                              |           |                                                                       |
| `reset_counters`    |        | Unknown                                              |           | Resets the boot counter and wdt counters                              |
| `reset_wdt`         | x      | Not functioning: wdts have not been initialized      | SP20      | Resets dedicated WDT -                                                |
| `config_cmd`        | x      | Not functioning as expected                          | SP20      | Restores default config                                               |
| `config_get`        | x      | Working                                              | SP20      | Returns current config (`eps_config_t` struct)                        |
| `config_set`        | x      | Not functioning as expected                          | SP20      | Sets config                                                           |
| `hard_reset`        | x      | Not functioning as expected: HITL connectivity fails | SP20      | Hard resets the P31u, including cycling the permanent outputs         |
| `config2_cmd`       |        | Unknown                                              |           | Controls the config2 system                                           |
| `config2_get`       | x      | Working                                              | SP20      | Returns the current config2                                           |
| `config2_set`       |        | Unknown                                              |           | Sends a config2 to the P31u, need to send `config2_cmd(2)` to save it |

 ### High-level functions
 Since the functions above aren't the most user-friendly, they have been packaged into easy-to-use commands which are listed below:
 
 | Function          | Tested | Status                                                          | Test Date | Short Description                                                                          |
|-------------------|--------|-----------------------------------------------------------------|-----------|--------------------------------------------------------------------------------------------|
| `pulse`           |        | Unknown                                                         |           | Pulses a P31u channel on for x milliseconds                                                |
| `pulse_pi`        |        | Unknown                                                         |           | Pulses a RPi channel `HIGH` for x milliseconds                                             |
| `electrolyzer`    | x      | Working                                                         | SP20      | Turns the electrolyzer on/off                                                              |
| `solenoid`        | x      | Unknown: most seems to work, but can't view GPIO pins from HITL | SP20      | Spikes the solenoid with 20V for x milliseconds and then holds it at 5V for y milliseconds |
| `glowplug`        | x      | Working                                                         | SP20      | Pulses glowplug for x milliseconds                                                         |
| `burnwire`        | x      | Working                                                         | SP20      | Turns on burnwires for x seconds                                                           |
| `comms`           |        | Unknown                                                         |           | Enables/disables voltage boost for the comms                                               |
| `comms_amplifier` |        | Unknown                                                         |           | Turns the comms on/off                                                                     |
| `adjust_string`   |        | Legacy: Unused                                                  |           |                                                                                            |
| `nasa_demo`       |        | Legacy                                                          |           |                                                                                            |
| `chamber`         |        | Legacy                                                          |           |                                                                                            |
 
 
 
 ## Example
 Here are the software steps to running commands on the P31u:
 
 - Import `power_controller` 
 - Initialize a `Power` object:
 ```python
from power_controller import *
gom = Power()
```

Now, you are able to run any of the above functions as shown here:
 ```python
gom.displayAll()
gom.set_single_output(3,1,0)
...
```

