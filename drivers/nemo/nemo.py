#!/usr/bin/env python3

"""
Python interface to NEMO (Netron Experiment in Moon Orbit). Intended for use on Raspberry Pi.
(c) 2020-2021 Los Alamos National Laboratory v3.4
"""

import logging
import os
import datetime
from pathlib import Path
from time import sleep

import pigpio
from adafruit_blinka.agnostic import board_id
if board_id != 'GENERIC_LINUX_PC':
    import board
    import busio


class I2CTransactionFailure(Exception):
    pass


class I2CDevice:
    """
    Simple class to represent I2C devices.
    """

    def __init__(self, dev_addr=0x13, log=True):
        """Class constructor"""
        if board_id != 'GENERIC_LINUX_PC':
            self._bus = busio.I2C(board.SCL, board.SDA)
        else:
            self._bus = None
        self._dev_addr = dev_addr

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

    def _read_register(self, reg_address, size):
        """Read registers from device"""
        values = bytearray(size)
        while not self._bus.try_lock():
            pass
        try:
            self._bus.writeto_then_readfrom(self._dev_addr, bytes([reg_address]), values)
        except OSError as error:
            self._bus.unlock()
            if error.errno == 121:
                raise I2CTransactionFailure('Read failure')
            else:
                raise
        self._bus.unlock()

        if self._log is not None:
            self._log.info(f'_read_register(0x{reg_address:02X}, {size}): {values}')
        return list(values)

    def _write_register(self, reg_address, values):
        """Read registers to device"""
        if self._log is not None:
            self._log.info(f'_write_register(0x{reg_address:02X}, {values})')
        while not self._bus.try_lock():
            pass
        try:
            self._bus.writeto(self._dev_addr, bytes([reg_address] + values))
        except OSError as error:
            self._bus.unlock()
            if error.errno == 121:
                raise I2CTransactionFailure('Write failure')
            else:
                raise
        self._bus.unlock()


class Domino(I2CDevice):
    """
    Class to abstract common functions for both Domino detectors.
    """

    # Important values and conversions
    DET_TEMP_TO_C       = (1.0 / 0x0100)
    BIAS_TO_V           = (2.5 / 0xFF)
    THRESHOLD_TO_V      = (0.5 / 0xFF)

    DET_MFG_SN_DICT = {
        0x00000163DAFF: '00581',
        0x00000163D712: '00745',
        0x00000163F373: '01678',
        0x00000163EE67: '01854',
        0x00000163F300: '01867',
        0x00000163DB47: '01896',
        0x00000163DD67: '01898',
        0x00000164B257: '02478',
        0x00000163D9E4: '02653',
        0x00000163D5D6: 'A201',
    }

    def __init__(self, dev_addr, reg_sn, reg_temp, reg_bias, reg_threshold, reg_bin_0, log=True):
        I2CDevice.__init__(self, dev_addr, log=log)
        self._reg_sn = reg_sn
        self._reg_temp = reg_temp
        self._reg_bias = reg_bias
        self._reg_threshold = reg_threshold
        self._reg_bin_0 = reg_bin_0

    @classmethod
    def temp_to_c(cls, temp_int16):
        """Convert temperature from int16 to degreess C"""
        return cls.DET_TEMP_TO_C * temp_int16

    @classmethod
    def bias_to_v(cls, bias_uint8):
        """Convert bias from uint8 to voltage"""
        return cls.BIAS_TO_V * bias_uint8

    @classmethod
    def v_to_bias(cls, bias_v):
        """Convert bias voltage to raw int8"""
        return int(bias_v / cls.BIAS_TO_V)

    @classmethod
    def threshold_to_v(cls, threshold_uint8):
        """Convert threshold from uint8 to voltage"""
        return cls.THRESHOLD_TO_V * threshold_uint8

    @classmethod
    def v_to_threshold(cls, threshold_v):
        """Convert bias voltage to raw int8"""
        return int(threshold_v / cls.THRESHOLD_TO_V)

    @classmethod
    def serial_number_to_mfg(cls, serial_number):
        """Convert raw serial to manufacture inscribed serial number"""
        sn = int.from_bytes(serial_number, byteorder='big')
        if sn in cls.DET_MFG_SN_DICT.keys():
            return cls.DET_MFG_SN_DICT[sn]
        return None

    @property
    def serial_number(self):
        """Domino Detector Serial Number"""
        return self._read_register(self._reg_sn, 6)

    @property
    def mfg_serial_number(self):
        """Detector manufacture inscribed serial number.
        Returns None if unknown"""
        return self.serial_number_to_mfg(self.serial_number)

    @property
    def temp_int16(self):
        """Domino Detector temperature as INT16"""
        vals = bytearray(self._read_register(self._reg_temp, 2))
        return int.from_bytes(vals, byteorder='little', signed=True)

    @property
    def temp_c(self):
        """Domino Detector temperature in degrees Celsius"""
        return self.temp_to_c(self.temp_int16)

    @property
    def bias_uint8(self):
        """Domino Detector bias as raw uint8"""
        return self._read_register(self._reg_bias, 1)[0]

    @bias_uint8.setter
    def bias_uint8(self, bias):
        val = int(bias)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self._reg_bias, [val])

    @property
    def bias_v(self):
        """Domino bias in volts"""
        return self.bias_to_v(self.bias_uint8)

    @bias_v.setter
    def bias_v(self, bias):
        self.bias_uint8 = self.v_to_bias(bias)

    @property
    def threshold_uint8(self):
        """Domino threshold as raw uint8"""
        return self._read_register(self._reg_threshold, 1)[0]

    @threshold_uint8.setter
    def threshold_uint8(self, threshold):
        val = int(threshold)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self._reg_threshold, [val])

    @property
    def threshold_v(self):
        """Domino threshold in volts"""
        return self.threshold_to_v(self.threshold_uint8)

    @threshold_v.setter
    def threshold_v(self, threshold):
        self.threshold_uint8 = self.v_to_threshold(threshold)

    @property
    def bins(self):
        """Get histogram bins for detector.
        Returns a list of bins from smallest width to largest."""
        bins = self._read_register(self._reg_bin_0, 32)
        bins += self._read_register(self._reg_bin_0 + 32, 32)
        return bins


class Nemo(I2CDevice):
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

    ASSEMBLY_DET_DICT = {
        'ETU1': {'det0_mfg_serial_number': '01854', 'det1_mfg_serial_number': '01678'},
        'FU1': {'det0_mfg_serial_number': '01867', 'det1_mfg_serial_number': '01898'},
        'FU2': {'det0_mfg_serial_number': '00581', 'det1_mfg_serial_number': '00745'},
    }

    def __init__(self, dev_addr=0x13, reset_gpio_ch=5, log=True):
        self._reset_gpio_ch = reset_gpio_ch
        self._pi = pigpio.pi()

        # setup reset line GPIO
        self._pi.set_mode(self._reset_gpio_ch, pigpio.OUTPUT)
        self.release_from_reset()

        I2CDevice.__init__(self, dev_addr, log=log)

        # setup detectors
        self.det0 = Domino(self._dev_addr, self.REG_D0_SN0, self.REG_D0_TEMP_L, self.REG_D0_BIAS,
                           self.REG_D0_THRESHOLD, self.REG_D0_BIN_0, log=log)

        self.det1 = Domino(self._dev_addr, self.REG_D1_SN0, self.REG_D1_TEMP_L, self.REG_D1_BIAS,
                           self.REG_D1_THRESHOLD, self.REG_D1_BIN_0, log=log)

    @classmethod
    def serial_number_to_assembly_name(cls, serial_number):
        """Assembly name string, based on serial_number.
        Returns None if unknown"""
        if serial_number in cls.ASSEMBLY_NAME_DICT.keys():
            return cls.ASSEMBLY_NAME_DICT[serial_number]
        return None

    def release_from_reset(self):
        """De-assert reset GPIO pin to allow NEMO PIC to run"""
        self._pi.write(self._reset_gpio_ch, 1)

    def hold_in_reset(self):
        """Assert reset GPIO pin to NEMO PIC in reset"""
        self._pi.write(self._reset_gpio_ch, 0)

    def software_reboot(self):
        """Initiates a reboot of the NEMO microcontroller
        User should wait a bit (~0.1 seconds) after calling this function before
        attempting to interact with NEMO again"""
        self._write_register(self.REG_REBOOT, [self.REBOOT_KEY])

    @property
    def assembly_name(self):
        """Assembly name string, based on serial_number.
        Returns '' if unknown"""
        return self.serial_number_to_assembly_name(self.serial_number)

    def self_test(self, verbose=False):
        """Performs a basic self-test of the hardware.
        If verbose=False, returns True if all tests pass, False otherwise.
        If verbose=True, returns of dict of test names and pass/fail boolean
        (True=Pass, False=Fail)."""

        test_results = {}

        test_results['firmware_revision_valid'] = (self.firmware_revision >= 3)

        test_results['assembly_name_valid'] = (self.assembly_name != '')

        test_results['det_en_valid'] = (self.det_enable == [True, True])

        # clock_inc_valid
        clock_start = self.clock
        sleep(5)
        clock_stop = self.clock
        clock_delta = (0x10000 + clock_stop - clock_start) % 0x10000
        test_results['clock_inc_valid'] = (abs(clock_delta - 5.0) <= 1.0)

        # det0_sn_valid
        if self.assembly_name in self.ASSEMBLY_DET_DICT.keys():
            test_results['det0_sn_valid'] = (
                self.det0.mfg_serial_number
                == self.ASSEMBLY_DET_DICT[self.assembly_name]['det0_mfg_serial_number'])
        elif self.det0.mfg_serial_number in Domino.DET_MFG_SN_DICT.keys():
            test_results['det0_sn_valid'] = True
        else:
            test_results['det0_sn_valid'] = False

        # det1_sn_valid
        if self.assembly_name in self.ASSEMBLY_DET_DICT.keys():
            test_results['det1_sn_valid'] = (
                self.det1.mfg_serial_number
                == self.ASSEMBLY_DET_DICT[self.assembly_name]['det1_mfg_serial_number'])
        elif self.det1.mfg_serial_number in Domino.DET_MFG_SN_DICT.keys():
            test_results['det1_sn_valid'] = True
        else:
            test_results['det1_sn_valid'] = False

        # det0_temp_valid
        if self.det0.mfg_serial_number is not None:
            test_results['det0_temp_valid'] = (self.det0.temp_c != 0.0)
        else:
            test_results['det0_temp_valid'] = False

        # det1_temp_valid
        if self.det1.mfg_serial_number is not None:
            test_results['det1_temp_valid'] = (self.det1.temp_c != 0.0)
        else:
            test_results['det1_temp_valid'] = False

        # config change
        original_bin_width = self.bin_width
        original_bin_0_min_width = self.bin_0_min_width

        self.bin_width = 1
        self.bin_0_min_width = 10

        if self.bin_width != 1:
            test_results['config_change'] = False
        elif self.bin_0_min_width != 10:
            test_results['config_change'] = False
        else:
            test_results['config_change'] = True

        # config revert
        self.bin_width = original_bin_width
        self.bin_0_min_width = original_bin_0_min_width

        if self.bin_width != original_bin_width:
            test_results['config_revert'] = False
        elif self.bin_0_min_width != original_bin_0_min_width:
            test_results['config_revert'] = False
        else:
            test_results['config_revert'] = True

        # return results
        if verbose:
            return test_results

        return False not in test_results.values()

    @property
    def firmware_revision(self):
        """NEMO microcontroller firmware revision number."""
        return self._read_register(self.REG_FW_REV, 1)[0]

    @property
    def serial_number(self):
        """Unique serial number for NEMO assembly.
        Comes from serial number programming resistors on NEMO PCB."""
        return self._read_register(self.REG_NEMO_SN, 1)[0]

    @property
    def last_reset(self):
        """Bitfield indicating the cause of last reset of the NEMO microcontroller.
        See Confluence or PIC24F RCON register for decoding."""
        vals = self._read_register(self.REG_LAST_RESET_L, 2)
        return vals[0] + (vals[1] << 8)

    @property
    def det_enable_uint8(self):
        """Detector enable status as uint8 bitfield, bit number corresponds to detector number"""
        return self._read_register(self.REG_DET_ENABLE, 1)[0] & 0b00000011

    @det_enable_uint8.setter
    def det_enable_uint8(self, det_en_uint8):
        det_en_uint8 &= 0b00000011
        self._write_register(self.REG_DET_ENABLE, [det_en_uint8])

    @property
    def det_enable(self):
        """list of detector enable status, starting with detector 0."""
        val = self.det_enable_uint8
        det0_en = ((val & 0b00000001) == 0b00000001)
        det1_en = ((val & 0b00000010) == 0b00000010)
        return [det0_en, det1_en]

    @det_enable.setter
    def det_enable(self, det_en):
        val = 0
        if det_en[0]:
            val |= 0b00000001
        if det_en[1]:
            val |= 0b00000010
        self.det_enable_uint8 = val

    @property
    def clock(self):
        """NEMO microcontroller seconds clock.
        Starts at 0 at power up or after reset. Rolls over after 0xFFFF."""
        vals = self._read_register(self.REG_CLOCK_L, 2)
        return vals[0] + (vals[1] << 8)

    @property
    def rate_width_min(self):
        """Minimum pulse width for rate in microseconds.
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_RATE_WIDTH_MIN, 1)[0]

    @rate_width_min.setter
    def rate_width_min(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_RATE_WIDTH_MIN, [val])

    @property
    def rate_width_max(self):
        """Maximum pulse width for rate in microseconds.
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_RATE_WIDTH_MAX, 1)[0]

    @rate_width_max.setter
    def rate_width_max(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_RATE_WIDTH_MAX, [val])

    @property
    def bin_width(self):
        """Bin width in microseconds for all detector and veto bins
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_BIN_WIDTH, 1)[0]

    @bin_width.setter
    def bin_width(self, val):
        val = int(val)
        val = max(val, 0x01)
        val = min(val, 0xFF)
        self._write_register(self.REG_BIN_WIDTH, [val])

    @property
    def bin_0_min_width(self):
        """Minimum pulse width in microseconds for bin 0 for all detectors and veto bins.
        Valid range: 0 to 255us."""
        return self._read_register(self.REG_BIN_0_MIN_WIDTH, 1)[0]

    @bin_0_min_width.setter
    def bin_0_min_width(self, val):
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
    def rate_interval(self):
        """Rate accumulation interval.
        How long each rate accumulates for in seconds.
        Valid range 1 to 255 seconds."""
        return self._read_register(self.REG_RATE_INTERVAL, 1)[0]

    @rate_interval.setter
    def rate_interval(self, val):
        val = int(val)
        val = max(val, 0x01)
        val = min(val, 0xFF)
        self._write_register(self.REG_RATE_INTERVAL, [val])

    @property
    def veto_threshold_min(self):
        """Minimum time in microseconds between pulses for a pair of pulses to be
        considered a veto count."""
        return self._read_register(self.REG_VETO_THRESHOLD_MIN, 1)[0]

    @veto_threshold_min.setter
    def veto_threshold_min(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_VETO_THRESHOLD_MIN, [val])

    @property
    def veto_threshold_max(self):
        """Maximum time in microseconds between pulses for a pair of pulses to be
        considered a veto count."""
        return self._read_register(self.REG_VETO_THRESHOLD_MAX, 1)[0]

    @veto_threshold_max.setter
    def veto_threshold_max(self, val):
        val = int(val)
        val = max(val, 0x00)
        val = min(val, 0xFF)
        self._write_register(self.REG_VETO_THRESHOLD_MAX, [val])

    @property
    def rate_available(self):
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

        while self.rate_available >= 3:
            rate_data_raw = self._read_register(
                self.REG_RATE,
                min(self.rate_available, 30))

            det0_rates.extend(rate_data_raw[0::3])
            det1_rates.extend(rate_data_raw[1::3])
            veto_rates.extend(rate_data_raw[2::3])

        return (det0_rates, det1_rates, veto_rates)

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
        """Get and print configuration"""
        print('Configuration:')
        print(f'  firmware_revision:  {nemo.firmware_revision}')
        print(f'  serial_number:      {nemo.serial_number} ({nemo.assembly_name})')
        print(f'  det_enable:         {nemo.det_enable}')
        print(f'  det0.serial_number: {nemo.det0.serial_number} ({nemo.det0.mfg_serial_number})')
        print(f'  det0.bias:          {nemo.det0.bias_v:5.3f} V ({nemo.det0.bias_uint8})')
        print(f'  det0.threshold:     {nemo.det0.threshold_v:5.3f} V ({nemo.det0.threshold_uint8})')
        print(f'  det1.serial_number: {nemo.det1.serial_number} ({nemo.det1.mfg_serial_number})')
        print(f'  det1.bias:          {nemo.det1.bias_v:5.3f} V ({nemo.det1.bias_uint8})')
        print(f'  det1.threshold:     {nemo.det1.threshold_v:5.3f} V ({nemo.det1.threshold_uint8})')
        print(f'  rate_width_min:     {nemo.rate_width_min}')
        print(f'  rate_width_max:     {nemo.rate_width_max}')
        print(f'  bin_width:          {nemo.bin_width}')
        print(f'  bin_0_min_width:    {nemo.bin_0_min_width}')
        print(f'  rate_interval:      {nemo.rate_interval}')
        print(f'  veto_threshold_min: {nemo.veto_threshold_min}')
        print(f'  veto_threshold_max: {nemo.veto_threshold_max}')
        print('')

    def print_self_test():
        """Perform and print results of self_test"""
        print('Performing self test...')
        self_test_results = nemo.self_test(verbose=True)

        if False not in self_test_results.values():
            print('Self Test: (Pass)')
        else:
            print('Self Test: (**Fail**)')

        for key, val in self_test_results.items():
            print(f'  {key}: {val}')
        print('')

    print_config()

    print_self_test()

    while True:
        print((nemo.clock, nemo.det0.temp_c, nemo.det1.temp_c, nemo.last_reset,
              nemo.rate_available))
        print('')
        print(f'Rate Data: {nemo.rate_data}')
        print('')
        print(f'Detector 0 Historgram: {nemo.det0.bins}')
        print(f'Detector 1 Historgram: {nemo.det1.bins}')
        print(f'Veto Historgram:       {nemo.veto_bins}')
        print('')
        nemo.reset_bins()

        sleep(2)
