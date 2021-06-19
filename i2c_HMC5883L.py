#!/usr/bin/env python

# i2c_HMC5883L.py
# 2015-04-01
# Public Domain

import time
import struct
import sys

import pigpio # http://abyz.co.uk/rpi/pigpio/python.html

if __name__ == "__main__":
   BUS=1

   HMC5883L_I2C_ADDR=0x0d

   RUNTIME=60.0

   pi=pigpio.pi('192.168.178.229')

   h = pi.i2c_open(BUS, HMC5883L_I2C_ADDR)
   print(h)

   if h >= 0: # Connected OK?

      # Initialise HMC5883L.
      pi.i2c_write_byte_data(h, 0x00, 0x75)  # CRA 75Hz.
      pi.i2c_write_byte_data(h, 0x02, 0x00)  # Mode continuous reads.

      read = 0

      start_time = time.time()

      while (time.time()-start_time) < RUNTIME:

         # 0x3 = X MSB, 0x4 = X LSB
         # 0x5 = Y MSB, 0x6 = Y LSB
         # 0x7 = Z MSB, 0x8 = Z LSB

         # > = big endian

         (s, b) = pi.i2c_read_i2c_block_data(h, 0x03, 6)

         if s >= 0:
            (x, y, z) = struct.unpack('>3h', memoryview(b))
            print("{} {} {}".format(x, y, z))
            read += 1

      pi.i2c_close(h)

   pi.stop()

   print(read, read/RUNTIME)

