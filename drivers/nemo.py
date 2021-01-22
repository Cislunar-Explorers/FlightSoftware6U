#!/usr/bin/env python3

"""
Python interface to NEMO (Netron Experiment in Moon Orbit). Intended for use on Raspberry Pi.
(c) 2020-2021 Los Alamos National Laboratory v1.8
"""

import logging
import os
import datetime
from pathlib import Path
from time import sleep

import board
import busio
import RPi.GPIO as GPIO


class Nemo:
    """
    Interface to NEMO microcontroller register map.
    """

    # Register addresses
    REG_FW_REV          = 0x00

    REG_NEMO_SN         = 0x01

    REG_D0_SN0          = 0x02
    REG_D0_SN1          = 0x03
    REG_D0_SN2          = 0x04
    REG_D0_SN3          = 0x05
    REG_D0_SN4          = 0x06
    REG_D0_SN5          = 0x07

    REG_D1_SN0          = 0x08
    REG_D1_SN1          = 0x09
    REG_D1_SN2          = 0x0A
    REG_D1_SN3          = 0x0B
    REG_D1_SN4          = 0x0C
    REG_D1_SN5          = 0x0D

    REG_LAST_RESET_L    = 0x0E
    REG_LAST_RESET_H    = 0x0F

    REG_REBOOT          = 0x10

    REG_DET_ENABLE      = 0x11

    REG_CLOCK_L         = 0x12
    REG_CLOCK_H         = 0x13

    REG_D0_TEMP_L       = 0x14
    REG_D0_TEMP_H       = 0x15

    REG_D1_TEMP_L       = 0x16
    REG_D1_TEMP_H       = 0x17

    REG_D0_BIAS         = 0x18

    REG_D1_BIAS         = 0x19

    REG_D0_THRESHOLD    = 0x1A

    REG_D1_THRESHOLD    = 0x1B

    REG_RATE_WIDTH_MIN  = 0x1C
    REG_RATE_WIDTH_MAX  = 0x1D

    REG_BIN_WIDTH       = 0x1E
    REG_BIN_0_MIN_WIDTH = 0x1F

    REG_RESET_BINS      = 0x20

    REG_RATE_INTERVAL   = 0x21

    REG_VETO_THRESHOLD_MIN  = 0x22
    REG_VETO_THRESHOLD_MAX  = 0x23

    REG_RATE_AVAILABLE_L    = 0x30
    REG_RATE_AVAILABLE_H    = 0x31

    REG_RATE            = 0x32

    REG_D0_BIN_0        = 0x40
    REG_D0_BIN_63       = 0x7F

    REG_D1_BIN_0        = 0x80
    REG_D1_BIN_63       = 0xBF

    REG_VETO_BIN_0      = 0xC0
    REG_VETO_BIN_63     = 0xFF

    # Important values and conversions
    REBOOT_KEY          = 0x81

    BIAS_TO_V           = (2.5 / 0xFF)
    THRESHOLD_TO_V      = (0.5 / 0xFF)

    ASSEMBLY_NAME_DICT = {
        0: 'ETU1',
        1: 'ETU2',
        2: 'FU1',
        3: 'FU2',
        4: 'ETU3',
        5: 'ETU4',
        6: 'ETU5',
        7: '',
        8: 'SW DEV',
        9: '',
        10: '',
        11: '',
        12: '',
        13: '',
        14: '',
        15: '',
    }

    DET_MFG_SN_DICT = {
        0x00000163DAFF: 581,
        0x00000163D712: 745,
        0x00000163F373: 1678,
        0x00000163EE67: 1854,
        0x00000163F300: 1867,
        0x00000163DB47: 1896,
        0x00000163DD67: 1898,
    }

    ASSEMBLY_DET_DICT = {
        'ETU1': {'det0_mfg_sn': 1854, 'det1_mfg_sn': 1678},
        'FU1': {'det0_mfg_sn': 1867, 'det1_mfg_sn': 1898},
        'FU2': {'det0_mfg_sn': 581, 'det1_mfg_sn': 745},
    }

    def __init__(self, dev_addr=0x13, reset_gpio_ch=5, log=True):
        # setup I2C bus
        self._bus = busio.I2C(board.SCL, board.SDA)
        self._dev_addr = dev_addr
        self._reset_gpio_ch = reset_gpio_ch

        # setup reset line GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._reset_gpio_ch, GPIO.OUT, initial=GPIO.HIGH)

        # setup logging
        if log:
            log_dir = os.path.join('.', 'logs')
            Path(log_dir).mkdir(parents=True, exist_ok=True)

            log_fname = os.path.join(
                log_dir,
                datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S_nemo.log'))

            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s %(levelname)s %(name)s %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S',
                handlers=[logging.FileHandler(log_fname)])

            self._log = logging.getLogger(__name__)
        else:
            self._log  = None

        self._cached_config = None
        self.get_config()

    def release_from_reset(self):
        """De-assert reset GPIO pin to allow NEMO PIC to run"""
        GPIO.output(self._reset_gpio_ch, GPIO.HIGH)

    def hold_in_reset(self):
        """Assert reset GPIO pin to NEMO PIC in reset"""
        GPIO.output(self._reset_gpio_ch, GPIO.LOW)
        self._cached_config = None

    def get_config(self, use_cached_values=True):
        """Get a dictinoary of all current config values from instrument.
        Set use_cached_values to False to force re-poll data from hardware."""

        if not use_cached_values or self._cached_config is None:
            self._cached_config = dict(
                firmware_revision=self._firmware_revision,
                serial_number=self._serial_number,
                det0_sn=self._det0_sn,
                det1_sn=self._det1_sn,
                det_enable=self._det_enable,
                det0_bias=self._det0_bias,
                det1_bias=self._det1_bias,
                det0_threshold=self._det0_threshold,
                det1_threshold=self._det1_threshold,
                rate_width_min=self._rate_width_min,
                rate_width_max=self._rate_width_max,
                bin_width=self._bin_width,
                bin_0_min_width=self._bin_0_min_width,
                rate_interval=self._rate_interval,
                veto_threshold_min=self._veto_threshold_min,
                veto_threshold_max=self._veto_threshold_max
            )

        self._log.info(f'get_config(): self._cached_config: {self._cached_config}')

        return self._cached_config

    def set_config(self, **kwargs):
        """Set instrument configuration."""

        # make sure self._cached_config has been built
        if self._cached_config is None:
            self.get_config()

        self._log.info(f'set_config({kwargs})')

        for key, value in kwargs.items():
            if key == 'det_enable':
                self._cached_config[key] = value
                self._det_enable = value

            elif key == 'det0_bias':
                self._cached_config[key] = value
                self._det0_bias = value

            elif key == 'det1_bias':
                self._cached_config[key] = value
                self._det1_bias = value

            elif key == 'det0_threshold':
                self._cached_config[key] = value
                self._det0_threshold = value

            elif key == 'det1_threshold':
                self._cached_config[key] = value
                self._det1_threshold = value

            elif key == 'rate_width_min':
                self._cached_config[key] = value
                self._rate_width_min = value

            elif key == 'rate_width_max':
                self._cached_config[key] = value
                self._rate_width_max = value

            elif key == 'bin_width':
                self._cached_config[key] = value
                self._bin_width = value

            elif key == 'bin_0_min_width':
                self._cached_config[key] = value
                self._bin_0_min_width = value

            elif key == 'rate_interval':
                self._cached_config[key] = value
                self._rate_interval = value

            elif key == 'veto_threshold_min':
                self._cached_config[key] = value
                self._veto_threshold_min = value

            elif key == 'veto_threshold_max':
                self._cached_config[key] = value
                self._veto_threshold_max = value

            else:
                self._log.warn(f'set_config(): invalid key "{key}"')
                print(f'set_config(): invalid key "{key}"')

        self._log.info(f'set_config(): self._cached_config: {self._cached_config}')

    def get_soh(self):
        """Get a dictinoary of instrument state of health."""

        return(dict(
            clock=self._clock,
            det0_temp=self._det0_temp,
            det1_temp=self._det1_temp,
            last_reset=self._last_reset,
            rate_available=self._rate_available
        ))

    def software_reboot(self):
        """Initiates a reboot of the NEMO microcontroller
        User should wait a bit (~0.1 seconds) after calling this function before
        attempting to interact with NEMO again"""
        self._write_register(self.REG_REBOOT, [self.REBOOT_KEY])
        self._cached_config = None

    @property
    def assembly_name(self):
        """Assembly name string, based on serial_number.
        Returns '' if unknown"""

        sn = self.get_config()['serial_number']

        if sn in self.ASSEMBLY_NAME_DICT.keys():
            return self.ASSEMBLY_NAME_DICT[sn]
        else:
            return ''

    def self_test(self, verbose=False):
        """Performs a basic self-test of the hardware.
        If verbose=False, returns True if all tests pass, False otherwise.
        If verbose=True, returns of dict of test names and pass/fail boolean
        (True=Pass, False=Fail)."""

        test_results = {}

        test_results['firmware_revision_valid'] = (self.get_config()['firmware_revision'] >= 3)

        test_results['assembly_name_valid'] = (self.assembly_name != '')

        test_results['det_en_valid'] = (self.get_config()['det_enable'] == [True, True])

        # clock_inc_valid
        clock_start = self.get_soh()['clock']
        sleep(5)
        clock_stop = self.get_soh()['clock']
        test_results['clock_inc_valid'] = (abs(clock_stop - clock_start - 5.0) <= 1.0)

        # det0_sn_valid
        if self.assembly_name in self.ASSEMBLY_DET_DICT.keys():
            test_results['det0_sn_valid'] = (
                self.det0_mfg_sn == self.ASSEMBLY_DET_DICT[self.assembly_name]['det0_mfg_sn'])
        elif self.det0_mfg_sn in self.DET_MFG_SN_DICT.keys():
            test_results['det0_sn_valid'] = True
        else:
            test_results['det0_sn_valid'] = False

        # det1_sn_valid
        if self.assembly_name in self.ASSEMBLY_DET_DICT.keys():
            test_results['det1_sn_valid'] = (
                self.det1_mfg_sn == self.ASSEMBLY_DET_DICT[self.assembly_name]['det1_mfg_sn'])
        elif self.det1_mfg_sn in self.DET_MFG_SN_DICT.keys():
            test_results['det1_sn_valid'] = True
        else:
            test_results['det1_sn_valid'] = False

        # det0_temp_valid
        if self.det0_mfg_sn is not None:
            test_results['det0_temp_valid'] = (self.get_soh()['det0_temp'] != 0.0)
        else:
            test_results['det0_temp_valid'] = False

        # det1_temp_valid
        if self.det1_mfg_sn is not None:
            test_results['det1_temp_valid'] = (self.get_soh()['det1_temp'] != 0.0)
        else:
            test_results['det1_temp_valid'] = False

        # config change
        original_config = self.get_config()

        self.set_config(bin_width=1, bin_0_min_width=10)

        if self.get_config()['bin_width'] != 1:
            test_results['config_change'] = False
        elif self.get_config()['bin_0_min_width'] != 10:
            test_results['config_change'] = False
        else:
            test_results['config_change'] = True

        # config revert
        self.set_config(bin_width=original_config['bin_width'],
                        bin_0_min_width=original_config['bin_0_min_width'])

        if self.get_config()['bin_width'] != original_config['bin_width']:
            test_results['config_revert'] = False
        elif self.get_config()['bin_0_min_width'] != original_config['bin_0_min_width']:
            test_results['config_revert'] = False
        else:
            test_results['config_revert'] = True

        # return results
        if verbose:
            return test_results
        else:
            return False not in test_results.values()

    def _det_mfg_sn(self, det_sn):
        """Converts a detector array serial into the manufacture inscribed serial number.
        Returns None if unknown"""

        sn = int.from_bytes(det_sn, byteorder='big')

        if sn in self.DET_MFG_SN_DICT.keys():
            return self.DET_MFG_SN_DICT[sn]
        else:
            return None

    @property
    def det0_mfg_sn(self):
        """Detector 0 manufacture inscribed serial number.
        Returns None if unknown"""
        return self._det_mfg_sn(self._det0_sn)

    @property
    def det1_mfg_sn(self):
        """Detector 1 manufacture inscribed serial number.
        Returns None if unknown"""
        return self._det_mfg_sn(self._det1_sn)

    def _read_register(self, reg_address, size):
        values = bytearray(size)
        while not self._bus.try_lock():
            pass
        self._bus.writeto_then_readfrom(self._dev_addr, bytes([reg_address]), values)
        self._bus.unlock()
        if self._log is not None:
            self._log.info(f'_read_register(0x{reg_address:02X}, {size}): {values}')
        return list(values)

    def _write_register(self, reg_address, values):
        if self._log is not None:
            self._log.info(f'_write_register(0x{reg_address:02X}, {values})')
        while not self._bus.try_lock():
            pass
        self._bus.writeto(self._dev_addr, bytes([reg_address] + values))
        self._bus.unlock()

    @property
    def _firmware_revision(self):
        """NEMO microcontroller firmware revision number."""
        return self._read_register(self.REG_FW_REV, 1)[0]

    @property
    def _serial_number(self):
        """Unique serial number for NEMO assembly.
        Comes from serial number programming resistors on NEMO PCB."""
        return self._read_register(self.REG_NEMO_SN, 1)[0]

    @property
    def _det0_sn(self):
        """Domino Detector 0 Serial Number"""
        return self._read_register(self.REG_D0_SN0, 6)

    @property
    def _det1_sn(self):
        """Domino Detector 1 Serial Number"""
        return self._read_register(self.REG_D1_SN0, 6)

    @property
    def _last_reset(self):
        """Bitfield indicating the cause of last reset of the NEMO microcontroller.
        See Confluence or PIC24F RCON register for decoding."""
        vals = self._read_register(self.REG_LAST_RESET_L, 2)
        return vals[0] + (vals[1] << 8)

    @property
    def _det_enable(self):
        """list of detector enable status, starting with detector 0."""
        val = self._read_register(self.REG_DET_ENABLE, 1)[0] & 0b00000011
        det0_en = ((val & 0b00000001) == 0b00000001)
        det1_en = ((val & 0b00000010) == 0b00000010)
        return [det0_en, det1_en]

    @_det_enable.setter
    def _det_enable(self, det_en):
        val = 0
        if det_en[0]:
            val |= 0b00000001
        if det_en[1]:
            val |= 0b00000010
        self._write_register(self.REG_DET_ENABLE, [val])

    @property
    def _clock(self):
        """NEMO microcontroller seconds clock.
        Starts at 0 at power up or after reset. Rolls over after 0xFFFF."""
        vals = self._read_register(self.REG_CLOCK_L, 2)
        return vals[0] + (vals[1] << 8)

    @property
    def _det0_temp(self):
        """Domino Detector 0 temperature in degrees Celsius"""
        vals = bytearray(self._read_register(self.REG_D0_TEMP_L, 2))
        return int.from_bytes(vals, byteorder='little', signed=True) / 256.0

    @property
    def _det1_temp(self):
        """Domino Detector 1 temperature in degrees Celsius"""
        vals = bytearray(self._read_register(self.REG_D1_TEMP_L, 2))
        return int.from_bytes(vals, byteorder='little', signed=True) / 256.0

    @property
    def _det0_bias(self):
        """Domino Detector 0 bias in volts"""
        val = self._read_register(self.REG_D0_BIAS, 1)[0]
        return self.BIAS_TO_V * val

    @_det0_bias.setter
    def _det0_bias(self, bias):
        val = int(bias / self.BIAS_TO_V)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_D0_BIAS, [val])

    @property
    def _det1_bias(self):
        """Domino Detector 1 bias in volts"""
        val = self._read_register(self.REG_D1_BIAS, 1)[0]
        return self.BIAS_TO_V * val

    @_det1_bias.setter
    def _det1_bias(self, bias):
        val = int(bias / self.BIAS_TO_V)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_D1_BIAS, [val])

    @property
    def _det0_threshold(self):
        """Domino Detector 0 threshold in volts"""
        val = self._read_register(self.REG_D0_THRESHOLD, 1)[0]
        return self.THRESHOLD_TO_V * val

    @_det0_threshold.setter
    def _det0_threshold(self, threshold):
        val = int(threshold / self.THRESHOLD_TO_V)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_D0_THRESHOLD, [val])

    @property
    def _det1_threshold(self):
        """Domino Detector 1 threshold in volts"""
        val = self._read_register(self.REG_D1_THRESHOLD, 1)[0]
        return self.THRESHOLD_TO_V * val

    @_det1_threshold.setter
    def _det1_threshold(self, threshold):
        val = int(threshold / self.THRESHOLD_TO_V)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_D1_THRESHOLD, [val])

    @property
    def _rate_width_min(self):
        """Minimum pulse width for rate in microseconds.
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_RATE_WIDTH_MIN, 1)[0]

    @_rate_width_min.setter
    def _rate_width_min(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_RATE_WIDTH_MIN, [val])

    @property
    def _rate_width_max(self):
        """Maximum pulse width for rate in microseconds.
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_RATE_WIDTH_MAX, 1)[0]

    @_rate_width_max.setter
    def _rate_width_max(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_RATE_WIDTH_MAX, [val])

    @property
    def _bin_width(self):
        """Bin width in microseconds for all detector and veto bins
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_BIN_WIDTH, 1)[0]

    @_bin_width.setter
    def _bin_width(self, val):
        val = int(val)
        val = max(val, 0x01)
        val = min(val, 0xFF)
        self._write_register(self.REG_BIN_WIDTH, [val])

    @property
    def _bin_0_min_width(self):
        """Minimum pulse width in microseconds for bin 0 for all detectors and veto bins.
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_BIN_0_MIN_WIDTH, 1)[0]

    @_bin_0_min_width.setter
    def _bin_0_min_width(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_BIN_0_MIN_WIDTH, [val])

    def reset_bins(self, reset_d0_bins=True, reset_d1_bins=True, reset_veto_bins=True):
        """Reset bin accumulator values to 0"""
        val = 0
        if reset_d0_bins:
            val |= 0b001
        if reset_d1_bins:
            val |= 0b010
        if reset_veto_bins:
            val |= 0b100

        self._write_register(self.REG_RESET_BINS, [val])

    @property
    def _rate_interval(self):
        """Rate accumulation interval.
        How long each rate accumulates for in seconds.
        Valid range 1 to 255 seconds."""
        return self._read_register(self.REG_RATE_INTERVAL, 1)[0]

    @_rate_interval.setter
    def _rate_interval(self, val):
        val = int(val)
        val = max(val, 0x01)
        val = min(val, 0xFF)
        self._write_register(self.REG_RATE_INTERVAL, [val])

    @property
    def _veto_threshold_min(self):
        """Minimum time in microseconds between pulses for a pair of pulses to be
        considered a veto count."""
        return self._read_register(self.REG_VETO_THRESHOLD_MIN, 1)[0]

    @_veto_threshold_min.setter
    def _veto_threshold_min(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_VETO_THRESHOLD_MIN, [val])

    @property
    def _veto_threshold_max(self):
        """Maximum time in microseconds between pulses for a pair of pulses to be
        considered a veto count."""
        return self._read_register(self.REG_VETO_THRESHOLD_MAX, 1)[0]

    @_veto_threshold_max.setter
    def _veto_threshold_max(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_VETO_THRESHOLD_MAX, [val])

    @property
    def _rate_available(self):
        """Number of rate entries available."""
        vals = self._read_register(self.REG_RATE_AVAILABLE_L, 2)
        return vals[0] + (vals[1] << 8)

    def reset_rate_data(self):
        """Reset (delete) all accumulated rate data from device"""
        self._write_register(self.REG_RATE_AVAILABLE_L, [0x00])

    @property
    def rate_data(self):
        """Get all available rate data from device.
        Returned as tuple of lists (det0_rates, det1_rates, veto_rates)"""

        det0_rates = []
        det1_rates = []
        veto_rates = []

        while self._rate_available >= 3:
            rate_data_raw = self._read_register(
                self.REG_RATE,
                min(self._rate_available, 30))

            det0_rates.extend(rate_data_raw[0::3])
            det1_rates.extend(rate_data_raw[1::3])
            veto_rates.extend(rate_data_raw[2::3])

        return (det0_rates, det1_rates, veto_rates)

    @property
    def det0_bins(self):
        """Get histogram bins for detector 0.
        Returns a list of bins from smallest width to largest."""
        bins = self._read_register(self.REG_D0_BIN_0, 32)
        bins += self._read_register(self.REG_D0_BIN_0 + 32, 32)
        return bins

    @property
    def det1_bins(self):
        """Get histogram bins for detector 1.
        Returns a list of bins from smallest width to largest."""
        bins = self._read_register(self.REG_D1_BIN_0, 32)
        bins += self._read_register(self.REG_D1_BIN_0 + 32, 32)
        return bins

    @property
    def veto_bins(self):
        """Get histogram bins for veto counts.
        Returns a list of bins from smallest width to largest."""
        bins = self._read_register(self.REG_VETO_BIN_0, 32)
        bins += self._read_register(self.REG_VETO_BIN_0 + 32, 32)
        return bins


if __name__ == "__main__":
    nemo = Nemo()

    def print_config():
        print('Configuration:')
        for k, v in nemo.get_config().items():
            if k == 'serial_number':
                print(f'  {k}: {v} ({nemo.assembly_name})')
            elif k == 'det0_sn':
                print(f'  {k}: {v} ({nemo.det0_mfg_sn})')
            elif k == 'det1_sn':
                print(f'  {k}: {v} ({nemo.det1_mfg_sn})')
            else:
                print(f'  {k}: {v}')
        print('')

    def print_self_test():
        print('Performing self test...')
        self_test_results = nemo.self_test(verbose=True)

        if False not in self_test_results.values():
            print('Self Test: (Pass)')
        else:
            print('Self Test: (**Fail**)')

        for k, v in self_test_results.items():
            print(f'  {k}: {v}')
        print('')

    print_config()

    print_self_test()

    while True:
        print(nemo.get_soh())
        print('')
        print(f'Rate Data: {nemo.rate_data}')
        print('')
        print(f'Detector 0 Historgram: {nemo.det0_bins}')
        print(f'Detector 1 Historgram: {nemo.det1_bins}')
        print(f'Veto Historgram:       {nemo.veto_bins}')
        print('')
        nemo.reset_bins()

        sleep(2)
