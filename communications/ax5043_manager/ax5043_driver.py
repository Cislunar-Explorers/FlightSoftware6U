from communications.ax5043_manager.ax5043_regs import Reg
from adafruit_bus_device.spi_device import SPIDevice
from typing import Dict


class Chunk:
    @staticmethod
    def from_bytes(buf):
        chunk_size = Chunk.check_length(buf)
        if not chunk_size:
            return (None, buf)

        rem = buf[chunk_size:]
        if buf[0] == 0x31:
            return (RssiChunk(buf[1]), rem)
        elif buf[0] == 0x52:
            return (FreqoffsChunk(buf[1], buf[2]), rem)
        elif buf[0] == 0x55:
            return (Antrssi2Chunk(buf[1], buf[2]), rem)
        elif buf[0] == 0x70:
            return (TimerChunk(buf[1], buf[2], buf[3]), rem)
        elif buf[0] == 0x73:
            return (RffreqoffsChunk(buf[1], buf[2], buf[3]), rem)
        elif buf[0] == 0x74:
            return (DatarateChunk(buf[1], buf[2], buf[3]), rem)
        elif buf[0] == 0x75:
            return (Antrssi3Chunk(buf[1], buf[2], buf[3]), rem)
        elif buf[0] == 0xE1:
            # length = buf[1]
            return (DataChunk(buf[2], buf[3:chunk_size]), rem)
        else:
            return (UnknownChunk(buf[0:chunk_size]), rem)

    @staticmethod
    def check_length(buf):
        assert len(buf) > 0
        top = buf[0] & 0xE0
        if top == 0x00:
            return True
        elif top == 0x20:
            return 2 * (len(buf) >= 2)
        elif top == 0x40:
            return 3 * (len(buf) >= 3)
        elif top == 0x60:
            return 4 * (len(buf) >= 4)
        elif top == 0xE0:
            if len(buf) < 1:
                return False
            else:
                length = buf[1]
                return (length + 2) * (len(buf) >= length + 2)
        else:
            raise RuntimeError("Invalid top bits: %02X" % top)

    @staticmethod
    def signed_byte(b):
        if b < 128:
            return b
        else:
            return b - 256


class RssiChunk(Chunk):
    def __init__(self, rssi):
        self.rssi = Chunk.signed_byte(rssi)


class FreqoffsChunk(Chunk):
    def __init__(self, freqoffs1, freqoffs0):
        self.freqoffs = (freqoffs1 << 8) | freqoffs0


class Antrssi2Chunk(Chunk):
    def __init__(self, rssi, bgndnoise):
        self.rssi = Chunk.signed_byte(rssi)
        self.bgndnoise = Chunk.signed_byte(bgndnoise)


class TimerChunk(Chunk):
    def __init__(self, timer2, timer1, timer0):
        self.timer = (timer2 << 16) | (timer1 << 8) | timer0


class RffreqoffsChunk(Chunk):
    def __init__(self, rffreqoffs2, rffreqoffs1, rffreqoffs0):
        self.rffreqoffs = (rffreqoffs2 << 16) | (rffreqoffs1 << 8) | rffreqoffs0


class DatarateChunk(Chunk):
    def __init__(self, datarate2, datarate1, datarate0):
        self.datarate = (datarate2 << 16) | (datarate1 << 8) | datarate0


class Antrssi3Chunk(Chunk):
    def __init__(self, antorssi2, antorssi1, antorssi0):
        # Programming manual suggests fields should be ANT0RSSI, ANT1RSSI, and BGNDNOISE
        self.antorssi2 = antorssi2
        self.antorssi1 = antorssi1
        self.antorssi0 = antorssi0


class DataChunk(Chunk):
    def __init__(self, flags, data):
        self.flags = flags
        self.data = data


class UnknownChunk(Chunk):
    def __init__(self, buf):
        self.buf = buf


# Note: CE0 is used as CS pin by Linux system calls (and is NOT held between
# Python calls, even in the same context), so all reads must use write_readinto.
class Ax5043:
    def __init__(self, bus: SPIDevice):
        self._bus = bus

    def execute(self, cmds: Dict[Reg, int]):
        last_addr = -2
        addr_wvals = None
        for addr, value in sorted(cmds.items()):
            if addr - last_addr != 1:
                # Write accumulated contiguous bytes
                if addr_wvals is not None:
                    with self._bus as spi:
                        spi.write(addr_wvals)

                # Initialize next write buffer
                if addr < 0x70:
                    addr_wvals = bytearray([0x80 | addr])
                else:
                    addr_wvals = bytearray([0xF0 | (addr >> 8), addr & 0xFF])

            addr_wvals.append(value)
            last_addr = addr

        # Write accumulated contiguous bytes
        if addr_wvals is not None:
            with self._bus as spi:
                spi.write(addr_wvals)
        # Enhancements: return status bits?

    def read(self, addr):
        if addr < 0x70:
            addr_wvals = bytearray([addr, 0])
        else:
            addr_wvals = bytearray([0x70 | (addr >> 8), addr & 0xFF, 0])
        rvals = bytearray(len(addr_wvals))
        with self._bus as spi:
            spi.write_readinto(addr_wvals, rvals)
        return rvals[-1]

    def read_16(self, addr):
        if addr < 0x70:
            addr_wvals = bytearray([addr, 0, 0])
        else:
            addr_wvals = bytearray([0x70 | (addr >> 8), addr & 0xFF, 0, 0])
        rvals = bytearray(len(addr_wvals))
        with self._bus as spi:
            spi.write_readinto(addr_wvals, rvals)
        return (rvals[-2] << 8) | rvals[-1]

    def set_pwrmode(self, mode):
        # Always sets REFEN, XOEN high
        self.execute({Reg.PWRMODE: 0x50 | mode})

    def reset(self):
        self.execute({Reg.PWRMODE: 0xE0})
        self.execute({Reg.PWRMODE: 0x50})

    def write_fifo(self, values):
        # Writing to FIFODATA does not auto-advance the SPI address pointer
        addr_wvals = bytearray([0x80 | Reg.FIFODATA]) + values
        rvals = bytearray(len(addr_wvals))
        with self._bus as spi:
            spi.write_readinto(addr_wvals, rvals)
        # TODO: Check FIFO status in rvals

    def write_fifo_data(self, data):
        # TODO: Document flags
        self.write_fifo(bytearray([0xE1, len(data) + 1, 0x13]) + data)

    def read_fifo(self, count):
        addr_wvals = bytearray([Reg.FIFODATA]) + bytearray(count)
        rvals = bytearray(len(addr_wvals))
        with self._bus as spi:
            spi.write_readinto(addr_wvals, rvals)
        return rvals[1:]

    def reg_dump(self) -> Dict[str, int]:
        """Read and return all registers on the AX5043"""
        return {
            reg.name: self.read(reg.value)
            for reg in Reg
            if self.read(reg.value) is not None
        }
