#!/usr/bin/env python3
'''this module defines a wrapper class
to control ftobler's robot arm'''
import re
import struct
import time
from functools import reduce
from typing import Union, List

import serial
from serial.tools import list_ports

###########################################################################
##### Robot Arm by ftobler python wrapper library                         #
##### Adapted from ftobler's Delphi controller program, and               #
##### HKU Maker Lab's Java controller program                             #
#####                                                                     #
##### Author: HUNG Lok Shun                                               #
##### Email: lshung@connect.hku.hk                                        #
##### License: CC BY-NC 3.0                                               #
##### Last modified: 2018-10-11 23:00                                     #
##### Python version: 3.6+ (3.7.0)                                        #
#####                                                                     #
##### ftobler Thingiverse link: https://www.thingiverse.com/thing:1718984 #
##### ftobler GitHub link: https://github.com/ftobler/robotArm            #
###########################################################################

###########################################################################
##### Library constants                                                   #
###########################################################################
# RobotArm pos constants
POS_HOME = "home"
POS_END_STOP = "end_stop"
POS_BOTTOM = "bottom"
POS_REST = "rest"

# RobotArm attachment type
ATTACHMENT_GRIPPER = 0
ATTACHMENT_FAN = 1

# Attachment gcode constants
GCODE_ENABLE_MOTORS = "M17"
GCODE_DISABLE_MOTORS = "M18"
GCODE_ENABLE_FAN = "M106"
GCODE_DISABLE_FAN = "M107"
GCODE_OPEN_GRIPPER = "M3 T{}"
GCODE_CLOSE_GRIPPER = "M5 T{}"

# Movement gcode constants
GCODE_G1 = "G1 X{} Y{} Z{}"


class RobotArm:
    '''A Robot Arm wrapper class, provides high-level movement methods'''
    #######################################################################
    # Static methods                                                      #
    #######################################################################
    @classmethod
    def get_default_serial_port(cls) -> serial.Serial:
        '''get an unopened USB serial port with default configuration, or throw error'''
        ser = serial.Serial()

        # find USB port
        ports = cls.get_available_serial_port_devices()
        usb_regex = re.compile("usb", flags=re.IGNORECASE)
        port = reduce(
            lambda acc, val: val,
            filter(usb_regex.search, ports)
        )
        if port is None:
            raise Exception('Cannot find USB port :(')

        # set port
        ser.port = port
        ser.baudrate = 9600
        ser.timeout = 2
        return ser

    @classmethod
    def get_available_serial_port_devices(cls) -> List[str]:
        '''get serial ports available on this device'''
        return [p.device for p in list_ports.comports()]

    @staticmethod
    def __encode_gcode(gcode: str) -> bytes:
        '''encode gcode as bytes'''
        return struct.pack(f"<{len(gcode)}s", gcode.encode(encoding="utf-8"))

    #######################################################################
    # Instance methods                                                    #
    #######################################################################
    # (0) Magic methods
    def __init__(
            self,
            attachment: int = ATTACHMENT_GRIPPER,
            serial_port: serial.Serial = None,
            use_default_serial_port: bool = True,
            start_connection: bool = True,
            verbose: bool = True
    ) -> None:
        '''constructor of RobotArm
        attachment: int - "ATTACHMENT_GRIPPER", "ATTACHMENT_FAN" | default: "ATTACHMENT_GRIPPER"
        serial_port: serial.Serial - serial port connecting to
        use_default_serial_port: bool - True to use a default serial port config
        start_connection: bool - True to open connection inside RobotArm constructor
        verbose: bool - True to print log

        use_default_serial_port option is disabled if serial_port is not None'''
        self.attachment = attachment
        self.serial_port = serial_port
        self._dont_print_logs = not verbose

        self.__already_init = False
        self.__x_pos = None
        self.__y_pos = None
        self.__z_pos = None

        self.coarse_step = 20
        self.fine_step = 5

        if serial_port is None and use_default_serial_port:
            self.serial_port = self.get_default_serial_port()

        if self.is_connection_open():
            self._init_seq()
        elif start_connection:
            self.open_connection()

    def __del__(self) -> None:
        '''destructor of RobotArm, closes serial port connection'''
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

    def __repr__(self) -> str:
        return f"RobotArm, {self.serial_port!r}"

    def __str__(self) -> str:
        return f"RobotArm, {self.serial_port!s}"

    # (1) State operation methods
    def open_connection(self) -> None:
        '''close serial port of RobotArm'''
        if self.serial_port is not None:
            try:
                self.print_log("Opening connection to serial port")
                self.serial_port.open()
            except OSError as err:
                self.__print_log(
                    f"error opening serial port: {err}", force=True)
            self._init_seq()
        else:
            raise Exception('No serial port defined')

    def close_connection(self) -> None:
        '''open serial port of RobotArm'''
        if self.serial_port is not None:
            self.print_log("Closing connection to serial port")
            self.serial_port.close()
        else:
            raise Exception('No serial port defined')

    def is_connection_open(self) -> bool:
        '''check if serial port of RobotArm is opened'''
        return False if self.serial_port is None else self.serial_port.is_open

    def is_connection_not_open(self) -> bool:
        '''check if serial port of RobotArm is not opened (alias method)'''
        return not self.is_connection_open()

    # (2) RobotArm high-level utilily methods
    def get_pos(self) -> tuple:
        '''current position of head'''
        return (self.__x_pos, self.__y_pos, self.__z_pos)

    def _init_seq(self) -> None:
        '''turn on stepper motor, initialize RobotArm position (protected method)'''
        if self.__already_init:
            self.print_log("already initialized")
            return

        # wait for connection to establish
        time.sleep(1.5)

        # initialization sequence
        self.gcode_enable_motors()
        self.gcode_pos_end_stop()
        self.gcode_attachment_low()

        self.__already_init = True
        self.print_log("initialization complete")

    # (2.1) Motor / Attachment utility methods
    def gcode_attachment_high(self) -> int:
        '''turn on attachment, return # bytes sent'''
        if self.attachment == ATTACHMENT_GRIPPER:
            return self.gcode_close_gripper()
        elif self.attachment == ATTACHMENT_FAN:
            return self.gcode_enable_fan()
        else:
            raise Exception(
                f"unimplemented attachment type: {self.attachment}"
            )

    def gcode_attachment_low(self) -> int:
        '''turn off attachment, return # bytes sent'''
        if self.attachment == ATTACHMENT_GRIPPER:
            return self.gcode_open_gripper()
        elif self.attachment == ATTACHMENT_FAN:
            return self.gcode_disable_fan()
        else:
            raise Exception(
                f"unimplemented attachment type: {self.attachment}"
            )

    def gcode_enable_motors(self) -> int:
        '''enable stepper motors and fan, return # bytes sent'''
        return self.__gcode_send(GCODE_ENABLE_MOTORS)

    def gcode_disable_motors(self) -> int:
        '''disable stepper motors and fan, return # bytes sent'''
        return self.__gcode_send(GCODE_DISABLE_MOTORS)

    # (2.1.1) Gripper methods
    def gcode_close_gripper(self, t: int = 10) -> int:
        '''open gripper / aux motor T in steps, return # bytes sent'''
        return self.__gcode_send(GCODE_OPEN_GRIPPER.format(t))

    def gcode_open_gripper(self, t: int = 10) -> int:
        '''close gripper / aux motor T in steps, return # bytes sent'''
        return self.__gcode_send(GCODE_CLOSE_GRIPPER.format(t))

    # (2.1.2) Fan methods
    def gcode_enable_fan(self) -> int:
        '''enable fan, return # bytes sent'''
        return self.__gcode_send(GCODE_ENABLE_FAN)

    def gcode_disable_fan(self) -> int:
        '''disable fan, return # bytes sent'''
        return self.__gcode_send(GCODE_DISABLE_FAN)

    # (2.2) Movement utility methods
    def gcode_send_pos(
            self,
            x_pos: Union[int, float] = None,
            y_pos: Union[int, float] = None,
            z_pos: Union[int, float] = None
    ) -> int:
        '''goto (x_pos, y_pos, z_pos), return # bytes sent'''
        x_pos = self.__x_pos if x_pos is None else x_pos
        y_pos = self.__y_pos if y_pos is None else y_pos
        z_pos = self.__z_pos if z_pos is None else z_pos
        x_str = f"{x_pos}" if isinstance(x_pos, int) else f"{x_pos:.1f}"
        y_str = f"{y_pos}" if isinstance(y_pos, int) else f"{y_pos:.1f}"
        z_str = f"{z_pos}" if isinstance(z_pos, int) else f"{z_pos:.1f}"
        self.__update_pos(x_pos, y_pos, z_pos)
        return self.__gcode_send(GCODE_G1.format(x_str, y_str, z_str))

    # (2.2.1) Predefined position movement methods
    def gcode_pos(self, to: str = POS_HOME) -> int:
        '''movement utility wrapper method
        to: POS_HOME, POS_END_STOP, POS_BOTTOM, POS_REST'''
        return {
            POS_HOME: self.gcode_pos_home,
            POS_END_STOP: self.gcode_pos_end_stop,
            POS_BOTTOM: self.gcode_pos_bottom,
            POS_REST: self.gcode_pos_rest
        }.get(to, lambda: 0)()

    def gcode_pos_home(self) -> int:
        '''go to home postition (0, 120, 120), return # bytes sent'''
        return self.gcode_send_pos(0, 120, 120)

    def gcode_pos_end_stop(self) -> int:
        '''go to end_stop position (0.0, 19.5, 134.0), return # bytes sent'''
        return self.gcode_send_pos(0.0, 19.5, 134.0)

    def gcode_pos_bottom(self) -> int:
        '''go to bottom position (0, 100, 0), return # bytes sent'''
        return self.gcode_send_pos(0, 100, 0)

    def gcode_pos_rest(self) -> int:
        '''go to rest position (0, 40, 70), return # bytes sent'''
        return self.gcode_send_pos(0, 40, 70)

    # (2.2.2) Step movement methods
    def set_coarse_step(self, val: int) -> None:
        '''set size of coarse step movement'''
        self.coarse_step = val

    def set_fine_step(self, val: int) -> None:
        '''set size of fine step movement'''
        self.fine_step = val

    def gcode_x_coarse_pos(self) -> int:
        '''move 1 coarse step in positive x direction'''
        return self.gcode_send_pos_delta(x_delta=self.coarse_step)

    def gcode_y_coarse_pos(self) -> int:
        '''move 1 coarse step in positive y direction'''
        return self.gcode_send_pos_delta(y_delta=self.coarse_step)

    def gcode_z_coarse_pos(self) -> int:
        '''move 1 coarse step in positive z direction'''
        return self.gcode_send_pos_delta(z_delta=self.coarse_step)

    def gcode_x_coarse_neg(self) -> int:
        '''move 1 coarse step in negative x direction'''
        return self.gcode_send_pos_delta(x_delta=self.coarse_step * -1)

    def gcode_y_coarse_neg(self) -> int:
        '''move 1 coarse step in negative y direction'''
        return self.gcode_send_pos_delta(y_delta=self.coarse_step * -1)

    def gcode_z_coarse_neg(self) -> int:
        '''move 1 coarse step in negative z direction'''
        return self.gcode_send_pos_delta(z_delta=self.coarse_step * -1)

    def gcode_x_fine_pos(self) -> int:
        '''move 1 fine step in positive x direction'''
        return self.gcode_send_pos_delta(x_delta=self.fine_step)

    def gcode_y_fine_pos(self) -> int:
        '''move 1 fine step in positive y direction'''
        return self.gcode_send_pos_delta(y_delta=self.fine_step)

    def gcode_z_fine_pos(self) -> int:
        '''move 1 fine step in positive z direction'''
        return self.gcode_send_pos_delta(z_delta=self.fine_step)

    def gcode_x_fine_neg(self) -> int:
        '''move 1 fine step in negative x direction'''
        return self.gcode_send_pos_delta(x_delta=self.fine_step * -1)

    def gcode_y_fine_neg(self) -> int:
        '''move 1 fine step in negative y direction'''
        return self.gcode_send_pos_delta(y_delta=self.fine_step * -1)

    def gcode_z_fine_neg(self) -> int:
        '''move 1 fine step in negative z direction'''
        return self.gcode_send_pos_delta(z_delta=self.fine_step * -1)

    # (2.2.3) Delta movement methods
    def gcode_send_pos_delta(
            self,
            x_delta: Union[int] = None,
            y_delta: Union[int] = None,
            z_delta: Union[int] = None
    ) -> int:
        '''move (dx, dy, dz) delta steps'''
        return self.gcode_send_pos(
            x_pos=self.__x_pos + x_delta if x_delta else self.__x_pos,
            y_pos=self.__y_pos + y_delta if y_delta else self.__y_pos,
            z_pos=self.__z_pos + z_delta if z_delta else self.__z_pos
        )

    # (3) RobotArm low-level utilily methods
    def gcode_send(self, gcode: str) -> int:
        '''low-level: send gcode to Arduino, update pos, return # bytes sent'''
        for seg in gcode.split(' '):
            if {
                    'X': lambda p: self.__update_pos(x_pos=p),  # return None
                    'Y': lambda p: self.__update_pos(y_pos=p),  # return None
                    'Z': lambda p: self.__update_pos(z_pos=p),  # return None
                    'G': lambda p: None                     # return None
            }.get(seg[0], lambda p: "break")(seg[1:]) is not None:
                break
        return self.__gcode_send(gcode)

    def __gcode_send(self, gcode: str) -> int:
        '''low-level: send gcode to Arduino, return # bytes sent (private method)'''
        if self.serial_port is None:
            raise Exception('No serial port defined')
        if self.is_connection_not_open():
            raise Exception('RobotArm serial port is not opened :(')

        ###################################
        # IMPORTANT - "\r\n" is required: #
        #   \r : Arduino starts execution #
        #   \n : skip iteration           #
        #                                 #
        # make sure to use Little-Endian  #
        ###################################
        gcode = f"{gcode}\r\n"

        w_bytes = self.serial_port.write(self.__encode_gcode(gcode))
        self.print_log(f"{gcode}", end='')
        return w_bytes

    # (4) Other utility methods
    def __update_pos(
            self,
            x_pos: Union[int, float, str] = None,
            y_pos: Union[int, float, str] = None,
            z_pos: Union[int, float, str] = None
    ) -> None:
        '''update (x_pos, y_pos, z_pos) position of RobotArm (private method)'''
        if x_pos is not None:
            if isinstance(x_pos, str):
                try:
                    x_pos = int(x_pos)
                except ValueError:
                    x_pos = float(x_pos)
            self.__x_pos = x_pos
        if y_pos is not None:
            if isinstance(y_pos, str):
                try:
                    y_pos = int(y_pos)
                except ValueError:
                    y_pos = float(y_pos)
            self.__y_pos = y_pos
        if z_pos is not None:
            if isinstance(z_pos, str):
                try:
                    z_pos = int(z_pos)
                except ValueError:
                    z_pos = float(z_pos)
            self.__z_pos = z_pos

    def print_log(self, msg: str, end: str = '\n') -> None:
        '''print to console'''
        return self.__print_log(msg, end=end, force=False)

    def __print_log(self, msg: str, end: str = '\n', force: bool = False) -> None:
        '''print to console (private method)'''
        if not self._dont_print_logs or force:
            print(f"RobotArm: {msg}", end=end)
