#!/usr/bin/env python

# i2c_ADXL345.py
# 2015-04-01
# Public Domain

import time
import struct
import sys
import math

import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html


class ADXL345:
    def __init__(self, pi, bus=1, addr=0x53):
        self.pi = pi
        self.h = pi.i2c_open(bus, addr)
        if self.h >= 0:
            pi.i2c_write_byte_data(self.h, 0x2d, 0)  # POWER_CTL reset.
            pi.i2c_write_byte_data(self.h, 0x2d, 8)  # POWER_CTL measure.
            pi.i2c_write_byte_data(self.h, 0x31, 0)  # DATA_FORMAT reset.
            pi.i2c_write_byte_data(self.h, 0x31, 11)  # DATA_FORMAT full res +/- 16g.

    def get_xyz_accel(self, factor = 21.9):
        (s, b) = self.pi.i2c_read_i2c_block_data(self.h, 0x32, 6)
        if s >= 0:
            (x, y, z) = struct.unpack('<3h', memoryview(b))
            x /= factor
            y /= factor
            z /= factor
            return x, y, z

    def get_accel_magnitude(self, xyz=None):
        if xyz:
            x, y, z = xyz
        else:
            x, y, z = self.get_xyz_accel()
        return math.sqrt(x**2 + y**2 + z**2)

    def get_accel_direction(self):
        x, y, z = self.get_xyz_accel()
        mag = self.get_accel_magnitude((x, y, z))
        x /= mag
        y /= mag
        z /= mag
        return x, y, z

    def close(self):
        self.pi.i2c_close(self.h)


class Magnetometer:
    def __init__(self, pi, bus=1, addr=0x1E):
        self.pi = pi
        self.h = pi.i2c_open(bus, addr)
        self.bus = bus
        self.addr = addr
        if self.h >= 0:
            self.pi.i2c_write_byte_data(self.h, 0x00, 0xF8)  # CRA 75Hz.
            self.pi.i2c_write_byte_data(self.h, 0x02, 0x00)  # Mode continuous reads.

    def get_magnetometers(self):
        (s, b) = self.pi.i2c_read_i2c_block_data(self.h, 0x03, 6)

        if s >= 0:
            (x, y, z) = struct.unpack('>3h', memoryview(b))
            return x,y,z




if __name__ == "__main__":

    pi = pigpio.pi('192.168.178.229')  # open local Pi
    imu = ADXL345(pi)
    while True:
        # print(imu.get_xyz_accel())
        # print(imu.get_accel_magnitude())
        print(imu.get_accel_direction())
        print()
        time.sleep(0.5)

