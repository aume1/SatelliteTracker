import numpy as np
import pigpio


class Servo:
    def __init__(self, pi: pigpio.pi, port, pulse_range=None, angle_range=None):
        if pulse_range is None:
            pulse_range = [550, 2050]
        if angle_range is None:
            angle_range = [0, 180]
        self.pi = pi
        self.port = port
        self.pi.set_mode(self.port, pigpio.OUTPUT)
        self.pulse_range = pulse_range
        self.angle_range = angle_range

    def set_angle(self, angle):
        if angle > max(self.angle_range):
            angle = max(self.angle_range)
        if angle < min(self.angle_range):
            angle = min(self.angle_range)
        pulse = self._angle_to_pulse(angle)
        self.pi.set_servo_pulsewidth(self.port, pulse)

    def _angle_to_pulse(self, angle):
        return np.interp(angle, [0, 180], self.pulse_range)


if __name__ == "__main__":
    import time
    pi = pigpio.pi("192.168.178.229")
    s1 = Servo(pi, 16)
    s2 = Servo(pi, 12)
    s1.set_angle(45)
    s2.set_angle(45)
    time.sleep(2)
