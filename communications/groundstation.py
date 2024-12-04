# Packing Imports

# Transmission Imports
import logging
import time
from datetime import datetime

from bitstring import BitArray

from fsw.communications.satellite_radio import Radio

from fsw.utils.constants import ZERO_WORD, ONE_WORD


class Groundstation:
    """A Mock ground station using the AX5043 (the same chip that we use on the spacecraft)as the radio."""

    def __init__(self):
        self.radio = Radio()
        self.driver = self.radio.driver
        self.mgr = self.radio.mgr
        self.last_transmit_time = datetime.today()
        self.last_telemetry_time = datetime.today()

    # Monitor radio health, request reset if faulted
    def monitorHealth(self):
        if self.mgr.is_faulted():
            logging.error('Radio manager faulted')
            self.mgr.reset_requested = True
            return None

    # Gets the signal from the radio board and returns it in a bytearray
    def receiveSignal(self):

        self.mgr.rx_enabled = True
        self.mgr.dispatch()
        self.monitorHealth()

        if self.mgr.outbox.empty():
            return None
        else:
            downlink = self.mgr.outbox.get()
            print('Inflated Bytes: ' + str(downlink))
            return bit_deflation(downlink, ZERO_WORD, ONE_WORD)

    # Downlink given bytearray to ground station
    def transmit(self, signal: bytes):
        self.mgr.tx_enabled = True
        self.mgr.inbox.put(signal)

        cycles = 0

        while True:

            self.mgr.dispatch()
            self.monitorHealth()

            cycles += 1
            # After 5s, break for clean shutdown
            # (TODO: use interrupt handler to ensure clean shutdown when killed,
            # or monitor transmitting state and exit when complete)
            if cycles >= 10:
                break
            time.sleep(1)

        self.mgr.tx_enabled = False
        self.mgr.rx_enabled = False
        self.mgr.dispatch()

        self.last_transmit_time = datetime.today()


def bit_deflation(downlink: bytearray, zero_word: bytes, one_word: bytes):

    deflatedBitArray = BitArray('', bin='')

    # Recover
    for i in range(len(downlink) // 2):
        byte = downlink[i * 2:(i * 2) + 2]

        if byte == zero_word:
            deflatedBitArray += '0b0'
        elif byte == one_word:
            deflatedBitArray += '0b1'
        else:
            # TODO try and figure out whether it's a 1 or 0
            pass

    return deflatedBitArray.bytes
