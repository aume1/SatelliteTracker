import pigpio
import numpy as np


class GPS:
    def __init__(self, pi, default_lat_long_alt= [-33.85670, 151.21521, 5]):
        super().__init__()
        self.pi = pi
        self.ser = self.pi.serial_open('/dev/serial0', 9600)
        self.previous_lat_long_alt = default_lat_long_alt
        self.values = np.zeros(8)
        self.time = 0
        self.lat = 0
        self.long = 0
        self.alt = 0
        self.update()

    def update(self):
        available = self.pi.serial_data_available(self.ser)
        if available == 0:
            return self.previous_lat_long_alt
        (b, d) = self.pi.serial_read(self.ser, available)
        if b <= 0:
            return self.previous_lat_long_alt
        try:
            d = d.decode('utf-8')
        except UnicodeDecodeError:
            return self.previous_lat_long_alt
        if '$GPRMC' in d:
            d = d.split('$GPRMC')[1].split(',')
            if len(d) >= 9 and d[3] != '':
                # print(d)
                self.time = d[1]

                lat_deg = int(d[3][:2])
                lat_min = float(d[3][2:])/60
                self.lat = lat_deg + lat_min
                if d[4] == 'S':
                    self.lat = -self.lat

                long_deg = int(d[5][:3])
                long_min = float(d[5][3:])/60
                self.long = long_deg + long_min
                if d[6] == "W":
                    self.long = -self.long

                self.alt = float(d[8])
                self.previous_lat_long_alt = self.lat, self.long, self.alt

        return self.previous_lat_long_alt

    def close(self):
        self.pi.serial_close(self.ser)


if __name__ == "__main__":
    print('GPS Testing')
    pi = pigpio.pi('192.168.178.229')
    print('connected to pi')
    gps = GPS(pi)
    import time
    while True:
        gps.update()
        print(gps.lat, gps.long)
        time.sleep(1.0)
