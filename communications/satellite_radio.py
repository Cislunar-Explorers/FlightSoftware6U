import logging
from typing import Any

from adafruit_blinka.agnostic import board_id

from utils.constants import ZERO_WORD, ONE_WORD

from drivers.device import Device, DeviceEnum

if board_id and board_id != "GENERIC_LINUX_PC":
    import board
    import busio
    from adafruit_bus_device.spi_device import SPIDevice

from communications.ax5043_manager.ax5043_driver import Ax5043
from communications.ax5043_manager.ax5043_manager import Manager
from bitstring import BitArray

from time import time, sleep


class Radio(Device):
    driver: Ax5043
    mgr: Manager

    def __init__(self):
        super().__init__(DeviceEnum.radio)
        self.last_transmit_time = time()

    def _connect_to_hardware(self):
        self.driver = Ax5043(
            SPIDevice(busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO))
        )
        self.mgr = Manager(self.driver)

    def _collect_telem(self) -> Any:
        state = self.mgr.state
        return state

    # Monitor radio health, request reset if faulted
    def monitorHealth(self):
        if self.mgr.is_faulted():
            logging.error("Radio manager faulted")
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
            return self.mgr.outbox.get()

    # Downlink given bytearray to ground station
    def transmit(self, signal: bytes):

        self.mgr.tx_enabled = True

        inflatedSignal = self.bit_inflation(signal, ZERO_WORD, ONE_WORD)
        print("Inflated Bytes: " + str(inflatedSignal))
        self.mgr.inbox.put(inflatedSignal)

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
            sleep(1)

        self.mgr.tx_enabled = False
        self.mgr.rx_enabled = False
        self.mgr.dispatch()

        self.last_transmit_time = time()

    def bit_inflation(
        self, downlink: bytes, zero_word: bytes, one_word: bytes
    ) -> bytearray:

        # Convert bytes to bits
        downlinkBitString = BitArray(bytes=downlink).bin

        inflatedByteArray = bytearray("", encoding="utf-8")

        # Add two bytes for every bit corresponding to the appropriate word
        for bit in downlinkBitString:
            if bit == "0":
                inflatedByteArray += zero_word
            else:
                inflatedByteArray += one_word

        return inflatedByteArray
