#!/usr/bin/env python3

"""
Interface between Cislunar FSW and Nemo v3.4
"""

import os
from threading import Thread, Event
import time
import datetime
from pathlib import Path
import logging

from . import nemo
from . import util


class NemoManager(Thread):
    """
    Object to manage NEMO instrument
    Saves NEMO data to files and provides high level command interface.
    """

    def __init__(self, data_dir='.', reset_gpio_ch=5):
        """NemoManager class constructor"""
        self._data_dir = data_dir

        self._nemo = nemo.Nemo(log=False, reset_gpio_ch=reset_gpio_ch)

        self._config = util.Configuration(config_fname=os.path.join(self._data_dir, 'config.json'))

        self._shutdown = Event()
        self._run = Event()

        Path(self._data_dir).mkdir(parents=True, exist_ok=True)

        self._config_file = util.RotatingFileManager(
            os.path.join(self._data_dir, 'config'),
            self._config.config_rotate_period)

        self._rate_data_file = util.RotatingFileManager(
            os.path.join(self._data_dir, 'rate_data'),
            self._config.rate_data_rotate_period)

        self._histogram_file = util.RotatingFileManager(
            os.path.join(self._data_dir, 'histogram'),
            self._config.histogram_rotate_period)

        self.set_config(**self._config.get_public_dict())

        self._t_last_config = None
        self._t_last_data = None

        Thread.__init__(self)
        self._run.set()
        self.start()

    @property
    def _sec_since_last_config(self):
        """
        Elapsed time in seconds since config was last written
        Returns None if unknown.
        """
        if self._t_last_config is not None:
            return (datetime.datetime.now() - self._t_last_config).total_seconds()
        return None

    @property
    def _sec_since_last_data(self):
        """
        Elapsed time in seconds since data (rate & histograms) were last written
        Returns None if unknown.
        """
        if self._t_last_data is not None:
            return (datetime.datetime.now() - self._t_last_data).total_seconds()
        return None

    def close(self):
        """Cleanly close the object"""
        self._shutdown.set()

    def pause(self):
        """
        Pause work in the main thread.
        Any work in progress will be completed first.
        """
        self._run.clear()

    def resume(self):
        """
        Resume work in the main thread.
        """
        self._run.set()

    def run(self):
        """
        Method representing the thread's activity.
        Overrides the Thread class' run() method.
        """
        while not self._shutdown.is_set():
            if self._run.wait(timeout=10.0):
                try:
                    # if time to write config to file
                    if (self._sec_since_last_config is None
                            or (self._sec_since_last_config > self._config.config_write_period)):
                        self._config_file.write(bytes(util.ConfigPacket(self._nemo)))
                        self._t_last_config = datetime.datetime.now()
                        logging.info('Wrote config')

                    # if time to write rate data and histogram to file
                    if (self._sec_since_last_data is None
                            or (self._sec_since_last_data > self._config.data_write_period)):
                        self._rate_data_file.write(bytes(util.RateDataPacket(self._nemo)))
                        self._histogram_file.write(bytes(util.HistogramPacket(self._nemo)))
                        self._t_last_data = datetime.datetime.now()
                        logging.info('Wrote rate data and histogram')

                    time.sleep(0.25)

                except nemo.I2CTransactionFailure:
                    logging.error('Nemo I2C transaction failure in NemoManager tread')
                    time.sleep(1)

    def write_register(self, reg_address, values):
        """Direct write of register on NEMO. Allows low-level diagnostics on-orbit."""
        try:
            self._nemo._write_register(reg_address, values)
        except nemo.I2CTransactionFailure:
            logging.error('Nemo I2C transaction failure in NemoManager.write_register')

    def read_register(self, reg_address, size):
        """
        Direct read of register on NEMO. Allows low-level diagnostics on-orbit.
        Results are writen to file config_{datetime}_{reg_address}_{size}
        """
        try:
            result = self._nemo._read_register(reg_address, size)

            dt_str = datetime.datetime.now().strftime('%Y%m%dT%H%M%SZ')
            fname = os.path.join(
                self._data_dir,
                f'read_register_{dt_str}_{reg_address:02X}_{size:02X}')

            with open(fname, 'ab+') as file:
                file.write(bytes(result))
        except nemo.I2CTransactionFailure:
            logging.error('Nemo I2C transaction failure in NemoManager.read_register')

    def set_config(self, **kwargs):
        """Set writeable configuration parameters."""
        try:
            if 'det_enable_uint8' in kwargs:
                self._config.set(det_enable_uint8=kwargs['det_enable_uint8'])
                self._nemo.det_enable_uint8 = kwargs['det_enable_uint8']

            if 'det0_bias_uint8' in kwargs:
                self._config.set(det0_bias_uint8=kwargs['det0_bias_uint8'])
                self._nemo.det0.bias_uint8 = kwargs['det0_bias_uint8']

            if 'det1_bias_uint8' in kwargs:
                self._config.set(det1_bias_uint8=kwargs['det1_bias_uint8'])
                self._nemo.det1.bias_uint8 = kwargs['det1_bias_uint8']

            if 'det0_threshold_uint8' in kwargs:
                self._config.set(det0_threshold_uint8=kwargs['det0_threshold_uint8'])
                self._nemo.det0.threshold_uint8 = kwargs['det0_threshold_uint8']

            if 'det1_threshold_uint8' in kwargs:
                self._config.set(det1_threshold_uint8=kwargs['det1_threshold_uint8'])
                self._nemo.det1.threshold_uint8 = kwargs['det1_threshold_uint8']

            if 'rate_width_min' in kwargs:
                self._config.set(rate_width_min=kwargs['rate_width_min'])
                self._nemo.rate_width_min = kwargs['rate_width_min']

            if 'rate_width_max' in kwargs:
                self._config.set(rate_width_max=kwargs['rate_width_max'])
                self._nemo.rate_width_max = kwargs['rate_width_max']

            if 'bin_width' in kwargs:
                self._config.set(bin_width=kwargs['bin_width'])
                self._nemo.bin_width = kwargs['bin_width']

            if 'bin_0_min_width' in kwargs:
                self._config.set(bin_0_min_width=kwargs['bin_0_min_width'])
                self._nemo.bin_0_min_width = kwargs['bin_0_min_width']

            if 'rate_interval' in kwargs:
                self._config.set(rate_interval=kwargs['rate_interval'])
                self._nemo.rate_interval = kwargs['rate_interval']

            if 'veto_threshold_min' in kwargs:
                self._config.set(veto_threshold_min=kwargs['veto_threshold_min'])
                self._nemo.veto_threshold_min = kwargs['veto_threshold_min']

            if 'veto_threshold_max' in kwargs:
                self._config.set(veto_threshold_max=kwargs['veto_threshold_max'])
                self._nemo.veto_threshold_max = kwargs['veto_threshold_max']

            if 'config_write_period' in kwargs:
                self._config.set(config_write_period=kwargs['config_write_period'])

            if 'config_rotate_period' in kwargs:
                self._config.set(config_rotate_period=kwargs['config_rotate_period'])
                self._config_file.period = kwargs['config_rotate_period']

            if 'data_write_period' in kwargs:
                self._config.set(data_write_period=kwargs['data_write_period'])

            if 'rate_data_rotate_period' in kwargs:
                self._config.set(rate_data_rotate_period=kwargs['rate_data_rotate_period'])
                self._rate_data_file.period = kwargs['rate_data_rotate_period']

            if 'histogram_rotate_period' in kwargs:
                self._config.set(histogram_rotate_period=kwargs['histogram_rotate_period'])
                self._histogram_file.period = kwargs['histogram_rotate_period']

            self._config.save()
        except nemo.I2CTransactionFailure:
            logging.error('Nemo I2C transaction failure in NemoManager.set_config')

    def power_off(self):
        """Turn power off to Nemo (really just holds it in reset"""
        try:
            self._nemo.hold_in_reset()
        except nemo.I2CTransactionFailure:
            logging.error('Nemo I2C transaction failure in NemoManager.power_off')

    def power_on(self):
        """Turn power on to Nemo (really just release from reset"""
        try:
            self._nemo.release_from_reset()
        except nemo.I2CTransactionFailure:
            logging.error('Nemo I2C transaction failure in NemoManager.power_on')

    def reboot(self):
        """Reboot and reconfigure Nemo"""
        try:
            self._nemo.software_reboot()
            time.sleep(0.05)
            self.set_config(**self._config.get_public_dict())
        except nemo.I2CTransactionFailure:
            logging.error('Nemo I2C transaction failure in NemoManager.reboot')

    def process_rate_data(self, t_start, t_stop, decimation_factor):
        """Process already saved rate data into a lower resolution."""
        input_packets = util.RateDataPacket.from_file(
            os.path.join(self._data_dir, 'rate_data_*T*Z'),
            sc_time_min=t_start,
            sc_time_max=t_stop,
            sort=True)

        fname = f'lores_rate_data_{t_start}_{t_stop}_{decimation_factor}'
        with open(os.path.join(self._data_dir, fname), 'wb') as file:
            for i in range(0, len(input_packets), decimation_factor):
                output_packet = util.LoResRateDataPacket(input_packets[i:i + decimation_factor])
                file.write(bytes(output_packet))

    def process_histograms(self, t_start, t_stop, decimation_factor):
        """Process already saved histograms into a lower resolution."""
        input_packets = util.HistogramPacket.from_file(
            os.path.join(self._data_dir, 'histogram_*T*Z'),
            sc_time_min=t_start,
            sc_time_max=t_stop,
            sort=True)

        fname = f'lores_histogram_{t_start}_{t_stop}_{decimation_factor}'
        with open(os.path.join(self._data_dir, fname), 'wb') as file:
            for i in range(0, len(input_packets), decimation_factor):
                output_packet = util.LoResHistogramPacket(
                    input_packets[i:i + decimation_factor])

                file.write(bytes(output_packet))


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    try:
        nemo_mgr = NemoManager(os.path.expanduser('~/.cislunar-flight-software/nemo/'))

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        nemo_mgr.close()
