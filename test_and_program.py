#!/usr/bin/env python
import os
import serial
import subprocess
import sys
import time


AVRDUDE_CMD = [
    "avrdude",
    "-v",
    "-patmega32u4",
    "-cavrispmkii", 
    "-Ulock:w:0x3F:m",
    "-Uefuse:w:0xcb:m",
    "-Uhfuse:w:0xd8:m",
    "-Ulfuse:w:0xff:m",
    "-Uflash:w:production.hex:i",
]

def find_port():
    while True:
        ttys = [filename for filename in os.listdir("/dev")
                if filename.startswith("cu.")
                and not "luetooth" in filename]
        ttys.sort(key=lambda k:(k.startswith("cu."), k))
        if ttys:
            return "/dev/" + ttys[0]
        time.sleep(0.1)


def main():
    print "Calling avrdude..."
    result = subprocess.check_call(AVRDUDE_CMD)
    if result != 0:
        print "avrdude failed with return code %d; quitting." % (result,)
        sys.exit(result)

    print "Waiting for serial port..."
    time.sleep(9.0)
    portname = find_port()
    print "Guessing port is %r" % (portname,)

    print "Fetching test results..."
    out = open("testresults.txt", "a")

    port = serial.Serial(portname, 115200)
    while True:
        line = port.readline()
        print line,
        out.write(line)
        if line.startswith("All tests passed"):
            sys.exit(0)
        elif line.startswith("Selftest FAIL"):
            sys.exit(1)


if __name__ == '__main__':
    main()
