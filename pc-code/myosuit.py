#!/usr/bin/env python

from serial import Serial
import sys

class MyoSuit(Serial):
    CMD_SETLEDS = 1
    CMD_SETADDR = 2
    CMD_GETADCS = 3

    def __init__(self, *args, **kw):
        super(Serial, self).__init__(*args, **kw)

    def sendCmd(self, addr, cmd, data):
        if len(data) != 4:
            raise TypeError("Wrong amount of data")
        if 0xf0 & cmd or 0xf0 & addr:
            raise ValueError("addr and cmd should be right-aligned 4-bits")
        first_byte = (addr << 4) | cmd
        print "Sending [0x%x 0x%x 0x%x 0x%x 0x%x]" % (first_byte, data[0], data[1], data[2], data[3])
        self.write("%c%c%c%c%c" % (first_byte, data[0], data[1], data[2], data[3]))


    def setLeds(self, addr, leds):
        if len(leds) != 4:
            raise TypeError("Wrong number of values for LEDs")

        self.sendCmd(addr, self.CMD_SETLEDS, leds)

    def setAddr(self, addr, newAddr):
        self.sendCmd(addr, self.CMD_SETADDR, (newAddr << 4, 0, 0, 0))

    def getAdcs(self, addr):
        self.flushInput()
        self.sendCmd(addr, self.CMD_GETADCS, (0, 0, 0, 0))
        data = [ord(c) for c in self.read(5)]
        print "Read [0x%x 0x%x 0x%x 0x%x 0x%x]" % tuple(data)
        leds = [0,0,0,0]
        leds[0] = (data[0] << 2) | (data[1] >> 6)
        leds[1] = ((data[1] & 0x3f) << 4) | (data[2] >> 4)
        leds[2] = ((data[2] & 0x0f) << 6) | (data[3] >> 2)
        leds[3] = ((data[3] & 0x03) << 8) | data[4]
        return leds


if __name__ == "__main__":
    import colorsys, time

    print "Starting demo"
    suit = MyoSuit(sys.argv[1], sys.argv[2], timeout=10)

    while True:
        for hue in (x * 0.001 for x in range(0,1000,2)):
            color = [int(v * 254) for v in colorsys.hsv_to_rgb(hue, 1, 1)]
            suit.setLeds(0xf, (0, color[0], color[1], color[2]))
            vals = suit.getAdcs(0xf)
            print "ADC values [%d, %d, %d, %d]" % tuple(vals)
            time.sleep(0.05)
