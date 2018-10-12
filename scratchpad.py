import re
import serial
import serial.tools.list_ports
import struct
import time
from functools import reduce

###########################################################################
##### This file is for testing only, do not use                           #
#####                                                                     #
##### Author: HUNG Lok Shun                                               #
##### Email: lshung@connect.hku.hk                                        #
##### License: CC BY-NC 3.0                                               #
##### Last modified: 2018-10-11 23:00                                     #
##### Python version: 3.6+ (3.7.0)                                        #
###########################################################################

def to_gcode(cmd):
    return struct.pack(f"<{len(cmd)}s", cmd.encode(encoding="utf-8"))

ports = [p.device for p in serial.tools.list_ports.comports()]

usb_re = re.compile("usb", flags=re.IGNORECASE)
sel_port = reduce(
    lambda acc, val: val,
    filter(usb_re.search, ports)
)  # '/dev/cu.wchusbserial1410'

baudrates = [
    2400,
    4800,
    9600,
    38400,
    57600,
    115200
]

sel_baudrate = 9600


g_code = "G1 X0 Y120 Z120\r\n"

ser = serial.Serial()
ser.port = sel_port
ser.baudrate = sel_baudrate

ser.timeout = 2  # 2000ms
# ser.bytesize = serial.EIGHTBITS
# ser.stopbits = serial.STOPBITS_ONE
# ser.parity = serial.PARITY_NONE

# ser.xonxoff = False
# ser.rtscts = False
# ser.dsrdtr = False

ser.open()  # ser = serial.Serial('/dev/cu.wchusbserial1410', 9600)
time.sleep(1.5)
print("serial port opened")
ser.write(
    to_gcode("M17\r\n")
)
ser.write(
    to_gcode("G1 X0.0 Y19.5 Z134.0\r\n")
)
print("init")
time.sleep(1)

ser.write(
    to_gcode("G1 X0 Y120 Z120\r\n")
)

# ser.write(g_code.encode())  # ser.write("G1 X0 Y120 Z120\r\n".encode())
# ser.flush()

ser.write(
    to_gcode("M3 T10\r\n")
)
print("open")
time.sleep(1)
ser.write(
    to_gcode("M5 T10\r\n")
)
print("close")
time.sleep(1)

ser.close()
print("serial port closed")
