import ctypes
import pathlib

if __name__ == "__main__":
    # Load the shared library into ctypes
    libname = pathlib.Path().absolute() / "libAX5043.so"
    ax_lib = ctypes.CDLL(libname)

    def ax_read_reg(uint16 address):
      return ax_lib.ax5043_readReg(address)

    def ax_write_reg(uint16 address, char value):
      ax_lib.ax5043_writeReg(address, value)
      return

    def ax_write_preamble():
      ax_lib.ax5043_writePreamble()
      return

    def ax_write_packet():
      ax_lib.ax5043_writePacket()
      return