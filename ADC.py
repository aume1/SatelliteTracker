import pigpio
from time import sleep
import numpy as np


class ADC:
    def __init__(self, pi, spi_channel=0):
        super().__init__()
        self.pi = pi
        self.spi_channel = spi_channel
        self.adc = pi.spi_open(spi_channel, 3000000, 0)
        self.values = np.zeros(8)
        self.update()

    def __getitem__(self, item):
        return self.values[item]

    def update(self):
        for i in range(len(self.values)):
            count, data = self.pi.spi_xfer(self.adc, [1, (8 + i) << 4, 0])
            ret = ((data[1] << 8) | data[2]) & 0x3FF
            self.values[i] = ret

    def close(self):
        self.pi.spi_close(self.adc)


if __name__ == "__main__":
    print('ADC Testing')
    pi = pigpio.pi('192.168.178.229')
    print('connected to pi')
    adc = ADC(pi)
    from servo import Servo
    s1 = Servo(pi, 16)
    s2 = Servo(pi, 12)
    while True:
        try:
            adc.update()
            x = int(np.interp(adc[0], [0, 1024], [0, 180]))
            y = int(np.interp(adc[1], [0, 1024], [0, 180]))
            s1.set_angle(x)
            s2.set_angle(y)
            print(x, y)
            # print(adc.get(0), adc.get(1))
            sleep(0.05)
        except KeyboardInterrupt:
            adc.close()
