#!/usr/bin/env python

# i2c_ITG3205.py
# 2015-04-01
# Public Domain

import time
import struct
import sys
import numpy as np

import pigpio  # http://abyz.co.uk/rpi/pigpio/python.html

import threading

ITG_3205_WHO_AM_I = 0x00  # R/W
ITG_3205_SMPLRT_DIV = 0x15  # R/W
ITG_3205_DLPF_FS = 0x16  # R/W
ITG_3205_INT_CFG = 0x17  # R/W
ITG_3205_INT_STATUS = 0x1A  # R
ITG_3205_TEMP_OUT_H = 0x1B  # R
ITG_3205_TEMP_OUT_L = 0x1C  # R
ITG_3205_GYRO_XOUT_H = 0x1D  # R
ITG_3205_GYRO_XOUT_L = 0x1E  # R
ITG_3205_GYRO_YOUT_H = 0x1F  # R
ITG_3205_GYRO_YOUT_L = 0x20  # R
ITG_3205_GYRO_ZOUT_H = 0x21  # R
ITG_3205_GYRO_ZOUT_L = 0x22  # R
ITG_3205_PWR_MGM = 0x3E  # R/W

# DLPF_FS

ITG_3205_FS_SEL_2000_DEG_SEC = 0x18

ITG_3205_DLPF_CFG_256_8 = 0x00
ITG_3205_DLPF_CFG_188_1 = 0x01
ITG_3205_DLPF_CFG_98_1 = 0x02
ITG_3205_DLPF_CFG_42_1 = 0x03
ITG_3205_DLPF_CFG_20_1 = 0x04
ITG_3205_DLPF_CFG_10_1 = 0x05
ITG_3205_DLPF_CFG_5_1 = 0x06

# INT_CFG

ITG_3205_IC_ACTL = 0x80
ITG_3205_IC_OPEN = 0x40
ITG_3205_IC_LATCH_INT_EN = 0x20
ITG_3205_IC_INT_ANYRD_2CLEAR = 0x10
ITG_3205_IC_ITG_RDY_EN = 0x04
ITG_3205_IC_RAW_RDY_EN = 0x01

# INT_STATUS

ITG_3205_IS_ITG_RDY = 0x04
ITG_3205_IS_RAW_DATA_RDY = 0x01

# PWR_MGM

ITG_3205_PM_H_RESET = 0x80
ITG_3205_PM_SLEEP = 0x40
ITG_3205_PM_STBY_XG = 0x20
ITG_3205_PM_STBY_YG = 0x10
ITG_3205_PM_STBY_ZG = 0x08

ITG_3205_PM_CLK_SEL_INTERNAL = 0x00
ITG_3205_PM_CLK_SEL_X_GYRO = 0x01
ITG_3205_PM_CLK_SEL_Y_GYRO = 0x02
ITG_3205_PM_CLK_SEL_Z_GYRO = 0x03
ITG_3205_PM_CLK_SEL_EXT_32768 = 0x04
ITG_3205_PM_CLK_SEL_EXT_192 = 0x05


class Gyro:
    def __init__(self, pi, bus=1, addr=0x68):
        self.pi = pi
        self.bus = bus
        self.addr = addr
        self.h = pi.i2c_open(bus, addr)
        self.x_rot = 0
        self.y_rot = 0
        self.z_rot = 0
        self._xbias = 0
        self._ybias = 0
        self._zbias = 0
        self.weight = 1 / 16
        self.prev_time = time.time()

        pi.i2c_write_byte_data(self.h, ITG_3205_PWR_MGM, ITG_3205_PM_CLK_SEL_X_GYRO)
        pi.i2c_write_byte_data(self.h, ITG_3205_SMPLRT_DIV, 0x07)
        pi.i2c_write_byte_data(self.h, ITG_3205_DLPF_FS, ITG_3205_FS_SEL_2000_DEG_SEC | ITG_3205_DLPF_CFG_188_1)
        pi.i2c_write_byte_data(self.h, ITG_3205_INT_CFG, 0x00)

    def calibrate(self, n=100, dt=0.01):
        xs = []
        ys = []
        zs = []
        for _ in range(n):
            x, y, z, _ = self.update(update_pos=False)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            time.sleep(dt)
        self._xbias = np.median(xs)
        self._ybias = np.median(ys)
        self._zbias = np.median(zs)

    def update_raw(self):
        (s, b) = self.pi.i2c_read_i2c_block_data(self.h, ITG_3205_TEMP_OUT_H, 8)
        if s >= 0:
            (t, x, y, z) = struct.unpack('>4h', memoryview(b))
            return x, y, z, t

    def run(self):
        self.update()
        time.sleep(0.001)

    def update(self, update_pos=True):
        vals = self.update_raw()
        _t = time.time()
        dt = _t - self.prev_time
        if vals:
            x, y, z, t = vals
            x = x * self.weight - self._xbias
            y = y * self.weight - self._ybias
            z = z * self.weight - self._zbias
            if update_pos:
                self.x_rot += x * dt
                self.y_rot += y * dt
                self.z_rot += z * dt
            t = (35 + ((t + 13200) / 280.0))
            self.prev_time = _t
            return x, y, z, t
            # print("{:.1f} {} {} {}".format(t, x, y, z))

    def get_rotations(self):
        self.update()
        return self.x_rot, self.y_rot, self.z_rot




if __name__ == "__main__":
    BUS = 1

    ITG_3205_I2C_ADDR = 0x68

    RUNTIME = 60.0
    pi = pigpio.pi('192.168.178.229')  # open local Pi
    gyro = Gyro(pi)
    gyro.calibrate()
    # gyro.update()
    while True:
        print(gyro.get_rotations())
        # print(gyro.get_rotations())
        time.sleep(0.02)

    h = pi.i2c_open(BUS, ITG_3205_I2C_ADDR)

    if h >= 0:  # Connected OK?

        # Initialise ITG_3205.
        # 0x17 0x00

        read = 0

        start_time = time.time()

        while (time.time() - start_time) < RUNTIME:

            # 0x1B = T MSB, 0x1C = T LSB
            # 0x1D = X MSB, 0x1E = X LSB
            # 0x1F = Y MSB, 0x20 = Y LSB
            # 0x20 = Z MSB, 0x22 = Z LSB

            # > = big endian

            (s, b) = pi.i2c_read_i2c_block_data(h, ITG_3205_TEMP_OUT_H, 8)

            if s >= 0:
                (t, x, y, z) = struct.unpack('>4h', memoryview(b))
                t = 35 + ((t + 13200) / 280.0)
                print("{:.1f} {} {} {}".format(t, x, y, z))
                read += 1

        pi.i2c_close(h)

    pi.stop()

    print(read, read / RUNTIME)
