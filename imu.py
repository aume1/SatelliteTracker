import time
import math
import py_qmc5883l
import pigpio
import adafruit_bmp280
from i2c_ADXL345 import ADXL345
import numpy as np
from i2c_ITG3205 import Gyro


class IMU:
    def __init__(self, pi):
        self.gyro = Gyro(pi)
        self.accel = ADXL345(pi)
        self.mag = py_qmc5883l.QMC5883L(pi)
        rpy = list(self.get_roll_pitch_yaw())
        self._prev_time = time.time()

    def get_accel(self):
        return self.accel.get_xyz_accel()

    def get_gyro(self):
        return self.gyro.get_rotations()

    def get_mag(self):
        return self.mag.get_dir()

    def get_north(self):
        D = self.get_accel()
        D_mag = math.sqrt(D[0]**2 + D[1]**2 + D[2]**2)
        D = [x/D_mag for x in D]
        # D = [x for x in acc_unit]  # used to be negative, flipped sensor so it is positive now
        E = np.cross(D, self.get_mag())  # east is the cross-product of down and the direction of magnet
        e_mag = math.sqrt(E[0]**2 + E[1]**2 + E[2]**2)
        E /= e_mag
        N = np.cross(E, D)  # north is the cross-product of east and down
        n_mag = math.sqrt(N[0] ** 2 + N[1] ** 2 + N[2] ** 2)
        N /= n_mag
        return N

    def get_roll_pitch_yaw(self):
        x, y, z = self.get_accel()

        x_Buff = float(x)
        y_Buff = float(y)
        z_Buff = float(z)
        roll = 180 + math.atan2(y_Buff, z_Buff) * 57.3
        pitch = math.atan2((- x_Buff), math.sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3
        if roll > 180:
            roll -= 360
        yaw = self.mag.get_bearing()

        return roll, pitch, yaw


if __name__ == "__main__":
    pi = pigpio.pi('192.168.178.229')
    imu = IMU(pi)
    while True:
        print(imu.get_roll_pitch_yaw())
