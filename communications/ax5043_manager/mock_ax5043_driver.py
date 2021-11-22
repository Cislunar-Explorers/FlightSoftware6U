from communications.ax5043_manager.ax5043_driver import Ax5043

# Ideas:
# * Writes replace read_defaults
# * FIFO utilities


class MockAx5043(Ax5043):
    def __init__(self):
        super().__init__(None)
        self.read_defaults = rst_values.copy()
        self.read_queue = {}

    def execute(self, cmds):
        pass

    def read(self, addr):
        if addr in self.read_queue:
            return self.read_queue.pop(addr)
        else:
            return self.read_defaults[addr]

    def read_16(self, addr):
        return 0x0000

    def read_fifo(self, count):
        return bytearray(count)

    def write_fifo(self, values):
        pass


rst_values = {
    0x000: 0x51,
    0x001: 0x7D,  # changed from 0x7d to
    0x002: 0x60,
    0x003: 0xF7,
    0x004: 0x77,
    0x005: 0x00,
    0x006: 0x00,
    0x007: 0x00,
    0x008: 0x00,
    0x009: 0x00,
    0x00A: 0x00,
    0x00B: 0x00,
    0x00C: 0x11,
    0x00D: 0x2A,
    0x00E: 0x00,
    0x00F: 0x00,
    0x010: 0x08,
    0x011: 0x02,
    0x012: 0x00,
    0x014: 0xFF,
    0x015: 0xFF,
    0x016: 0xFF,
    0x017: 0xFF,
    0x018: 0x00,
    0x019: 0x62,
    0x01A: 0x00,
    0x01C: 0x00,
    0x01D: 0x01,
    0x020: 0x00,
    0x021: 0x08,
    0x022: 0x04,
    0x023: 0x87,
    0x024: 0x03,
    0x025: 0x06,
    0x026: 0x06,
    0x027: 0x00,
    0x028: 0x00,
    0x029: 0x00,
    0x02A: 0x00,
    0x02B: 0x00,
    0x02C: 0x00,
    0x02D: 0x00,
    0x02E: 0x00,
    0x02F: 0x00,
    0x030: 0x00,
    0x031: 0x00,
    0x032: 0x00,
    0x033: 0x00,
    0x034: 0x00,
    0x035: 0x00,
    0x036: 0x00,
    0x037: 0x00,
    0x038: 0x00,
    0x039: 0x00,
    0x03B: 0x08,
    0x03C: 0x39,
    0x03D: 0x34,
    0x03E: 0xCC,
    0x03F: 0xCD,
    0x040: 0x00,
    0x041: 0x00,
    0x042: 0x00,
    0x043: 0x00,
    0x045: 0x00,
    0x046: 0x00,
    0x047: 0x00,
    0x048: 0x00,
    0x049: 0x00,
    0x04A: 0x00,
    0x04B: 0x00,
    0x04D: 0x00,
    0x04E: 0x00,
    0x04F: 0x00,
    0x050: 0x00,
    0x051: 0x00,
    0x052: 0x00,
    0x053: 0x00,
    0x054: 0x00,
    0x055: 0x00,
    0x059: 0x00,
    0x05A: 0x00,
    0x05B: 0x00,
    0x068: 0x00,
    0x069: 0x00,
    0x06A: 0x00,
    0x06B: 0x00,
    0x06C: 0x00,
    0x06D: 0x00,
    0x06E: 0x00,
    0x100: 0x13,
    0x101: 0x27,
    0x102: 0x0D,
    0x103: 0x00,
    0x104: 0x3D,
    0x105: 0x8A,
    0x106: 0x00,
    0x107: 0x00,
    0x108: 0x9E,
    0x109: 0x00,
    0x10A: 0x16,
    0x10B: 0x87,
    0x10C: 0x00,
    0x10D: 0x80,
    0x10E: 0xFF,
    0x10F: 0x80,
    0x110: 0x00,
    0x111: 0x40,
    0x112: 0x00,
    0x113: 0x75,
    0x114: 0x04,
    0x115: 0x00,
    0x116: 0x00,
    0x117: 0x00,
    0x118: 0x00,
    0x120: 0xB4,
    0x121: 0x76,
    0x122: 0x00,
    0x123: 0x00,
    0x124: 0xF8,
    0x125: 0xF2,
    0x126: 0xC3,
    0x127: 0x0F,
    0x128: 0x1F,
    0x129: 0x0A,
    0x12A: 0x0A,
    0x12B: 0x46,
    0x12C: 0x00,
    0x12D: 0x20,
    0x12E: 0x16,
    0x12F: 0x88,
    0x130: 0xB4,
    0x131: 0x76,
    0x132: 0x00,
    0x133: 0x00,
    0x134: 0xF6,
    0x135: 0xF1,
    0x136: 0xC3,
    0x137: 0x0F,
    0x138: 0x1F,
    0x139: 0x0B,
    0x13A: 0x0B,
    0x13B: 0x46,
    0x13C: 0x00,
    0x13D: 0x20,
    0x13E: 0x18,
    0x13F: 0x88,
    0x140: 0xFF,
    0x141: 0x76,
    0x142: 0x00,
    0x143: 0x00,
    0x144: 0xF5,
    0x145: 0xF0,
    0x146: 0xC3,
    0x147: 0x0F,
    0x148: 0x1F,
    0x149: 0x0D,
    0x14A: 0x0D,
    0x14B: 0x46,
    0x14C: 0x00,
    0x14D: 0x20,
    0x14E: 0x1A,
    0x14F: 0x88,
    0x150: 0xFF,
    0x151: 0x76,
    0x152: 0x00,
    0x153: 0x00,
    0x154: 0xF5,
    0x155: 0xF0,
    0x156: 0xC3,
    0x157: 0x0F,
    0x158: 0x1F,
    0x159: 0x0D,
    0x15A: 0x0D,
    0x15B: 0x46,
    0x15C: 0x00,
    0x15D: 0x20,
    0x15E: 0x1A,
    0x15F: 0x88,
    0x160: 0x00,
    0x161: 0x00,
    0x162: 0x0A,
    0x163: 0x3D,
    0x164: 0x05,
    0x165: 0x00,
    0x166: 0x28,
    0x167: 0xF6,
    0x168: 0x00,
    0x169: 0x00,
    0x16A: 0x0F,
    0x16B: 0xFF,
    0x16C: 0x00,
    0x16D: 0x00,
    0x16E: 0x00,
    0x16F: 0x00,
    0x170: 0x00,
    0x171: 0x00,
    0x180: 0x12,
    0x181: 0x10,
    0x182: 0x03,
    0x183: 0x03,
    0x184: 0x00,
    0x188: 0x09,
    0x189: 0x77,
    0x200: 0x20,
    0x201: 0x00,
    0x202: 0x00,
    0x203: 0x00,
    0x204: 0x00,
    0x205: 0x00,
    0x206: 0x00,
    0x207: 0x00,
    0x208: 0x00,
    0x209: 0x00,
    0x20A: 0x00,
    0x20B: 0x00,
    0x210: 0x00,
    0x211: 0x00,
    0x212: 0x00,
    0x213: 0x00,
    0x214: 0x00,
    0x215: 0x00,
    0x216: 0x1F,
    0x218: 0x00,
    0x219: 0x00,
    0x21C: 0x00,
    0x21D: 0x00,
    0x21E: 0x0F,
    0x220: 0x32,
    0x221: 0x0A,
    0x223: 0x32,
    0x224: 0x14,
    0x225: 0x73,
    0x226: 0x39,
    0x227: 0x00,
    0x228: 0x00,
    0x229: 0x00,
    0x22A: 0x00,
    0x22B: 0x00,
    0x22C: 0x00,
    0x22D: 0x00,
    0x22E: 0x00,
    0x22F: 0x00,
    0x230: 0x00,
    0x231: 0x00,
    0x232: 0x00,
    0x233: 0x00,
    0x300: 0x00,
    0x301: 0x3F,
    0x308: 0x00,
    0x309: 0x00,
    0x310: 0x00,
    0x311: 0x00,
    0x312: 0x20,
    0x313: 0xC4,
    0x314: 0x61,
    0x315: 0xA8,
    0x316: 0x00,
    0x317: 0x00,
    0x318: 0x00,
    0x319: 0x00,
    0x330: 0x00,
    0x331: 0x00,
    0x332: 0x00,
    0xF00: 0x00,
    0xF08: 0x04,
    0xF0C: 0x00,
    0xF0D: 0x04,
    0xF10: 0x04,
    0xF11: 0x80,
    0xF18: 0x06,
    0xF1C: 0x04,
    0xF21: 0x20,
    0xF22: 0x54,
    0xF23: 0x50,
    0xF26: 0x88,
    0xF30: 0x3F,
    0xF31: 0xF0,
    0xF32: 0x3F,
    0xF33: 0xF0,
    0xF34: 0x0F,
    0xF35: 0x10,
    0xF44: 0x25,
    0xF5F: 0xE7,
    0xF72: 0x00,
}
