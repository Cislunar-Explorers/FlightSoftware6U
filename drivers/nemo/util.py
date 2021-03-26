#!/usr/bin/env python3

"""
Utilities for working with Nemo data v3.4
"""

import os
import datetime
import struct
import glob
import json

from .nemo import Nemo, Domino


class Configuration:
    """Stores configurable parameters"""
    _argdefaults = {
        "det_enable_uint8": 0b00000011,

        "det0_bias_uint8": 255,
        "det1_bias_uint8": 255,

        "det0_threshold_uint8": 141,
        "det1_threshold_uint8": 141,

        "rate_width_min": 0x00,
        "rate_width_max": 0xFF,

        "bin_width": 0x04,
        "bin_0_min_width": 0x00,

        "rate_interval": 10,

        "veto_threshold_min": 0x00,
        "veto_threshold_max": 0xFF,

        "config_write_period": 3600,
        "config_rotate_period": 86400,

        "data_write_period": 200,
        "rate_data_rotate_period": 3600,
        "histogram_rotate_period": 3600,
    }

    def __init__(self, config_fname='config.json', **kwargs):
        """
        Configuration class constructor

        Sets default values, then loads from file if available, then sets from
        """
        self._config_fname = config_fname

        # set defaults
        self.set(**self._argdefaults)

        # load from file
        self.load()

        # set provide values
        self.set(**kwargs)

    def set(self, **kwargs):
        """Set parameters using kwargs"""
        for attribute, value in kwargs.items():
            setattr(self, attribute, value)

    def get_public_dict(self):
        """Returns all public attirbutes of this object"""
        result = dict()
        for attribute, value in self.__dict__.items():
            if not attribute.startswith('_'):
                result[attribute] = value

        return result

    def load(self, new_config_fname=None):
        """Load config from file"""
        if new_config_fname is not None:
            self._config_fname = new_config_fname

        if os.path.exists(self._config_fname):
            with open(self._config_fname, 'r') as file:
                try:
                    self.set(**json.load(file))
                except json.decoder.JSONDecodeError:
                    pass

    def save(self):
        """Save current config back to file"""
        config_dict = self.get_public_dict()
        with open(self._config_fname, 'w') as file:
            json.dump(config_dict, file)


class RotatingFileManager:
    """
    Rotates through different file names based on current time and rotation period.
    """

    def __init__(self, basename, period, dt_format='_%Y%m%dT%H%M%SZ'):
        """
        RotatingFileManager class constructor
        :param basename: string base file name. May be either full path or relative.
        :param period: file rotation period in seconds
        """
        self._basename = basename
        self.period = period
        self._dt_format = dt_format

    def write(self, data):
        """Write data to current file."""
        with open(self._current_fname(), 'ab+') as file:
            file.write(data)

    def _current_fname(self):
        epoch_time = datetime.datetime.now().timestamp()
        epoch_time_rounded = int(epoch_time - (epoch_time % self.period))
        dt_rounded = datetime.datetime.fromtimestamp(epoch_time_rounded, tz=datetime.timezone.utc)
        return os.path.join(self._basename + dt_rounded.strftime(self._dt_format))


class NemoPacketBase:

    def __bytes__(self):
        return self.packet

    def __repr__(self):
        return f'{self.__class__.__qualname__}(sc_time=\'{self.sc_time}\',' \
            f'serial_number={self.serial_number})'

    @property
    def sc_dt(self):
        """spacecraft time as datetime object"""
        return datetime.datetime.fromtimestamp(self.sc_time, tz=datetime.timezone.utc)

    @property
    def assembly_name(self):
        """Assembly name string, based on serial_number.
        Returns None if unknown"""
        return Nemo.serial_number_to_assembly_name(self.serial_number)

    @classmethod
    def from_file(cls, filename, sc_time_min=None, sc_time_max=None, sort=False):
        """
        Return a list of Packet objects read from the given filename.
        :param: filename: Can contain wildcards to parse multiple files
        :para: sc_time_min: Minimum sc_time of packets to return (inclusive)
        :para: sc_time_max: Maximum sc_time of packets to return (inclusive)
        :para: sort: If true, return list of packets sorted by sc_time ascending
        """
        if sc_time_min is None:
            sc_time_min = 0

        if sc_time_max is None:
            sc_time_max = (2**32) - 1

        packets = []
        for name in glob.glob(filename):
            with open(name, 'rb') as file:
                data = file.read()
                for i in range(0, len(data), cls.PACKET_SIZE):
                    packet_bytes = data[i:(i + cls.PACKET_SIZE)]
                    if len(packet_bytes) == cls.PACKET_SIZE:
                        packet = cls(packet_bytes)
                        if packet.sc_time >= sc_time_min and packet.sc_time <= sc_time_max:
                            packets.append(packet)

        if sort:
            packets.sort(key=lambda x: x.sc_time)

        return packets


class ConfigPacket(NemoPacketBase):
    """Packet definition for storing and retrieving Configuration Data"""

    FORMAT_CODE = '>IB6B6BBH11B'
    PACKET_SIZE = struct.calcsize(FORMAT_CODE)

    def __init__(self, arg):
        """Constructor"""

        if isinstance(arg, bytes):
            self.packet = arg

            packet_unpacked = struct.unpack(self.FORMAT_CODE, self.packet)

            self.sc_time = packet_unpacked[0]
            self.serial_number = packet_unpacked[1]
            self.det0_sn = list(packet_unpacked[2:8])
            self.det1_sn = list(packet_unpacked[8:14])
            self.det_enable_uint8 = packet_unpacked[14]
            self.clock = packet_unpacked[15]
            self.det0_bias_uint8 = packet_unpacked[16]
            self.det1_bias_uint8 = packet_unpacked[17]
            self.det0_threshold_uint8 = packet_unpacked[18]
            self.det1_threshold_uint8 = packet_unpacked[19]
            self.rate_width_min = packet_unpacked[20]
            self.rate_width_max = packet_unpacked[21]
            self.bin_width = packet_unpacked[22]
            self.bin_0_min_width = packet_unpacked[23]
            self.rate_interval = packet_unpacked[24]
            self.veto_threshold_min = packet_unpacked[25]
            self.veto_threshold_max = packet_unpacked[26]
        else:
            nemo = arg

            self.sc_time = int(datetime.datetime.now().timestamp())
            self.serial_number = nemo.serial_number
            self.det0_sn = nemo.det0.serial_number
            self.det1_sn = nemo.det1.serial_number
            self.det_enable_uint8 = (int(nemo.det_enable[0])
                                     + (int(nemo.det_enable[1]) << 1))
            self.clock = nemo.clock
            self.det0_bias_uint8 = nemo.det0.bias_uint8
            self.det1_bias_uint8 = nemo.det1.bias_uint8
            self.det0_threshold_uint8 = nemo.det0.threshold_uint8
            self.det1_threshold_uint8 = nemo.det1.threshold_uint8
            self.rate_width_min = nemo.rate_width_min
            self.rate_width_max = nemo.rate_width_max
            self.bin_width = nemo.bin_width
            self.bin_0_min_width = nemo.bin_0_min_width
            self.rate_interval = nemo.rate_interval
            self.veto_threshold_min = nemo.veto_threshold_min
            self.veto_threshold_max = nemo.veto_threshold_max

            self.packet = struct.pack(
                self.FORMAT_CODE,
                self.sc_time,
                self.serial_number,
                *self.det0_sn,
                *self.det1_sn,
                self.det_enable_uint8,
                self.clock,
                self.det0_bias_uint8,
                self.det1_bias_uint8,
                self.det0_threshold_uint8,
                self.det1_threshold_uint8,
                self.rate_width_min,
                self.rate_width_max,
                self.bin_width,
                self.bin_0_min_width,
                self.rate_interval,
                self.veto_threshold_min,
                self.veto_threshold_max)

    @property
    def det0_mfg_sn(self):
        """Detector manufacture inscribed serial number.
        Returns None if unknown"""
        return Domino.serial_number_to_mfg_serial_number(self.det0_sn)

    @property
    def det1_mfg_sn(self):
        """Detector manufacture inscribed serial number.
        Returns None if unknown"""
        return Domino.serial_number_to_mfg_serial_number(self.det1_sn)

    @property
    def det0_bias_v(self):
        """Detector bias in volts"""
        return Domino.bias_to_v(self.det0_bias_uint8)

    @property
    def det1_bias_v(self):
        """Detector bias in volts"""
        return Domino.bias_to_v(self.det1_bias_uint8)

    @property
    def det0_threshold_v(self):
        """Detector threshold in volts"""
        return Domino.threshold_to_v(self.det0_threshold_uint8)

    @property
    def det1_threshold_v(self):
        """Detector threshold in volts"""
        return Domino.threshold_to_v(self.det1_threshold_uint8)

    def __eq__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return ((self.packet,
                 self.sc_time,
                 self.serial_number,
                 self.det0_sn,
                 self.det1_sn,
                 self.det_enable_uint8,
                 self.clock,
                 self.det0_bias_uint8,
                 self.det1_bias_uint8,
                 self.det0_threshold_uint8,
                 self.det1_threshold_uint8,
                 self.rate_width_min,
                 self.rate_width_max,
                 self.bin_width,
                 self.bin_0_min_width,
                 self.rate_interval,
                 self.veto_threshold_min,
                 self.veto_threshold_max)
                == (other.packet,
                    other.sc_time,
                    other.serial_number,
                    other.det0_sn,
                    other.det1_sn,
                    other.det_enable_uint8,
                    other.clock,
                    other.det0_bias_uint8,
                    other.det1_bias_uint8,
                    other.det0_threshold_uint8,
                    other.det1_threshold_uint8,
                    other.rate_width_min,
                    other.rate_width_max,
                    other.bin_width,
                    other.bin_0_min_width,
                    other.rate_interval,
                    other.veto_threshold_min,
                    other.veto_threshold_max))


class RateDataPacket(NemoPacketBase):
    """Packet definition for storing and retrieving Rate Data"""

    NUM_SAMPLES = 20
    FORMAT_CODE = '>IBHHhh' + ('B' * 3 * NUM_SAMPLES)
    PACKET_SIZE = struct.calcsize(FORMAT_CODE)

    def __init__(self, arg):
        """Constructor"""

        if isinstance(arg, bytes):
            self.packet = arg

            packet_unpacked = struct.unpack(self.FORMAT_CODE, self.packet)

            self.sc_time = packet_unpacked[0]
            self.serial_number = packet_unpacked[1]
            self.last_reset = packet_unpacked[2]
            self.clock = packet_unpacked[3]
            self.det0_temp_int16 = packet_unpacked[4]
            self.det1_temp_int16 = packet_unpacked[5]
            self.rate_data = [
                list(packet_unpacked[(6 + self.NUM_SAMPLES * 0):(6 + self.NUM_SAMPLES * 1)]),
                list(packet_unpacked[(6 + self.NUM_SAMPLES * 1):(6 + self.NUM_SAMPLES * 2)]),
                list(packet_unpacked[(6 + self.NUM_SAMPLES * 2):(6 + self.NUM_SAMPLES * 3)])]

        else:
            nemo = arg

            self.sc_time = int(datetime.datetime.now().timestamp())
            self.serial_number = nemo.serial_number
            self.last_reset = nemo.last_reset
            self.clock = nemo.clock
            self.det0_temp_int16 = nemo.det0.temp_int16
            self.det1_temp_int16 = nemo.det1.temp_int16

            rate_data = list(nemo.rate_data)
            for i in range(len(rate_data)):
                # trim execess old data
                rate_data[i] = rate_data[i][-self.NUM_SAMPLES:]

                # pad with zeros if not enough data
                rate_data[i] = rate_data[i] + ((self.NUM_SAMPLES - len(rate_data[i])) * [0])

            self.rate_data = rate_data

            self.packet = struct.pack(
                self.FORMAT_CODE,
                self.sc_time,
                self.serial_number,
                self.last_reset,
                self.clock,
                self.det0_temp_int16,
                self.det1_temp_int16,
                *self.rate_data[0],
                *self.rate_data[1],
                *self.rate_data[2])

    def __eq__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return ((self.packet,
                 self.sc_time,
                 self.serial_number,
                 self.last_reset,
                 self.clock,
                 self.det0_temp_int16,
                 self.det1_temp_int16,
                 self.rate_data)
                == (other.packet,
                    other.sc_time,
                    other.serial_number,
                    other.last_reset,
                    other.clock,
                    other.det0_temp_int16,
                    other.det1_temp_int16,
                    other.rate_data))


class LoResRateDataPacket(NemoPacketBase):
    """Packet definition for low-resolution Rate Data"""

    NUM_SAMPLES = 20
    FORMAT_CODE = '>IBHHBh' + ('H' * 3 * NUM_SAMPLES)
    PACKET_SIZE = struct.calcsize(FORMAT_CODE)

    def __init__(self, arg):
        """Constructor"""

        if isinstance(arg, bytes):
            self.packet = arg

            packet_unpacked = struct.unpack(self.FORMAT_CODE, self.packet)

            self.sc_time = packet_unpacked[0]
            self.serial_number = packet_unpacked[1]
            self.clock_0 = packet_unpacked[2]
            self.clock_1 = packet_unpacked[3]
            self.decimation_factor = packet_unpacked[4]
            self.det0_temp_int16 = packet_unpacked[5]
            self.rate_data = [
                list(packet_unpacked[(6 + self.NUM_SAMPLES * 0):(6 + self.NUM_SAMPLES * 1)]),
                list(packet_unpacked[(6 + self.NUM_SAMPLES * 1):(6 + self.NUM_SAMPLES * 2)]),
                list(packet_unpacked[(6 + self.NUM_SAMPLES * 2):(6 + self.NUM_SAMPLES * 3)])]
        else:
            packets = arg

            self.sc_time = packets[-1].sc_time
            self.serial_number = packets[-1].serial_number
            self.clock_0 = packets[0].clock
            self.clock_1 = packets[-1].clock
            self.decimation_factor = len(packets)
            self.det0_temp_int16 = packets[-1].det0_temp_int16
            self.rate_data = [[], [], []]

            for ch in range(3):
                time_ordered_rates = []
                for packet in reversed(packets):
                    time_ordered_rates += packet.rate_data[ch]

                downsampled_rates = []
                for i in range(0, len(time_ordered_rates), self.decimation_factor):
                    downsampled_rates.append(
                        sum(time_ordered_rates[i:(i + self.decimation_factor)]))

                self.rate_data[ch] = downsampled_rates

            self.packet = struct.pack(
                self.FORMAT_CODE,
                self.sc_time,
                self.serial_number,
                self.clock_0,
                self.clock_1,
                self.decimation_factor,
                self.det0_temp_int16,
                *self.rate_data[0],
                *self.rate_data[1],
                *self.rate_data[2])

    def __eq__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return ((self.packet,
                 self.sc_time,
                 self.serial_number,
                 self.clock_0,
                 self.clock_1,
                 self.decimation_factor,
                 self.det0_temp_int16,
                 self.rate_data)
                == (other.packet,
                    other.sc_time,
                    other.serial_number,
                    other.clock_0,
                    other.clock_1,
                    other.decimation_factor,
                    other.det0_temp_int16,
                    other.rate_data))


class HistogramPacket(NemoPacketBase):
    """Packet definition for storing and retrieving Histogram Data"""

    NUM_BINS = 64
    FORMAT_CODE = '>IBH' + ('B' * 3 * NUM_BINS)
    PACKET_SIZE = struct.calcsize(FORMAT_CODE)

    def __init__(self, arg):
        """Constructor"""

        if isinstance(arg, bytes):
            self.packet = arg

            packet_unpacked = struct.unpack(self.FORMAT_CODE, self.packet)

            self.sc_time = packet_unpacked[0]
            self.serial_number = packet_unpacked[1]
            self.clock = packet_unpacked[2]
            self.det0_bins = list(packet_unpacked[(3 + self.NUM_BINS * 0):(3 + self.NUM_BINS * 1)])
            self.det1_bins = list(packet_unpacked[(3 + self.NUM_BINS * 1):(3 + self.NUM_BINS * 2)])
            self.veto_bins = list(packet_unpacked[(3 + self.NUM_BINS * 2):(3 + self.NUM_BINS * 3)])

        else:
            nemo = arg

            self.sc_time = int(datetime.datetime.now().timestamp())
            self.serial_number = nemo.serial_number
            self.clock = nemo.clock
            self.det0_bins = nemo.det0.bins
            self.det1_bins = nemo.det1.bins
            self.veto_bins = nemo.veto_bins
            nemo.reset_bins()

            self.packet = struct.pack(
                self.FORMAT_CODE,
                self.sc_time,
                self.serial_number,
                self.clock,
                *self.det0_bins,
                *self.det1_bins,
                *self.veto_bins)

    def __eq__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return ((self.packet,
                 self.sc_time,
                 self.serial_number,
                 self.clock,
                 self.det0_bins,
                 self.det1_bins,
                 self.veto_bins)
                == (other.packet,
                    other.sc_time,
                    other.serial_number,
                    other.clock,
                    other.det0_bins,
                    other.det1_bins,
                    other.veto_bins))


class LoResHistogramPacket(NemoPacketBase):
    """Packet definition for low-resolution histogram data"""

    NUM_BINS = 64
    FORMAT_CODE = '>IBHHB' + ('H' * 3 * NUM_BINS)
    PACKET_SIZE = struct.calcsize(FORMAT_CODE)

    def __init__(self, arg):
        """Constructor"""

        if isinstance(arg, bytes):
            self.packet = arg

            packet_unpacked = struct.unpack(self.FORMAT_CODE, self.packet)

            self.sc_time = packet_unpacked[0]
            self.serial_number = packet_unpacked[1]
            self.clock_0 = packet_unpacked[2]
            self.clock_1 = packet_unpacked[3]
            self.decimation_factor = packet_unpacked[4]

            self.det0_bins = list(packet_unpacked[(5 + self.NUM_BINS * 0):(5 + self.NUM_BINS * 1)])
            self.det1_bins = list(packet_unpacked[(5 + self.NUM_BINS * 1):(5 + self.NUM_BINS * 2)])
            self.veto_bins = list(packet_unpacked[(5 + self.NUM_BINS * 2):(5 + self.NUM_BINS * 3)])

        else:
            packets = arg

            self.sc_time = packets[-1].sc_time
            self.serial_number = packets[-1].serial_number
            self.clock_0 = packets[0].clock
            self.clock_1 = packets[-1].clock
            self.decimation_factor = len(packets)

            self.det0_bins = self.NUM_BINS * [0]
            self.det1_bins = self.NUM_BINS * [0]
            self.veto_bins = self.NUM_BINS * [0]

            for packet in packets:
                for bin_num in range(self.NUM_BINS):
                    self.det0_bins[bin_num] += packet.det0_bins[bin_num]
                    self.det1_bins[bin_num] += packet.det1_bins[bin_num]
                    self.veto_bins[bin_num] += packet.veto_bins[bin_num]

            self.packet = struct.pack(
                self.FORMAT_CODE,
                self.sc_time,
                self.serial_number,
                self.clock_0,
                self.clock_1,
                self.decimation_factor,
                *self.det0_bins,
                *self.det1_bins,
                *self.veto_bins)

    def __eq__(self, other):
        if not isinstance(other, __class__):
            return NotImplemented
        return ((self.packet,
                 self.sc_time,
                 self.serial_number,
                 self.clock_0,
                 self.clock_1,
                 self.decimation_factor,
                 self.det0_bins,
                 self.det1_bins,
                 self.veto_bins)
                == (other.packet,
                    other.sc_time,
                    other.serial_number,
                    other.clock_0,
                    other.clock_1,
                    other.decimation_factor,
                    other.det0_bins,
                    other.det1_bins,
                    other.veto_bins))
