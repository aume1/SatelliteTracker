import requests
import time
import sgp4
from skyfield.api import EarthSatellite, S, E, wgs84, load
from skyfield.framelib import itrs
import skyfield.api
import urllib.request as urllib2


url = 'https://celestrak.com/NORAD/elements/stations.txt'
planets_file = 'de421.bsp'


class TLEManager:
    def __init__(self, target_sats=None, default_lat_long_alt=None):
        if default_lat_long_alt is None:
            default_lat_long_alt = [-33.85670, 151.21521, 5]
        if target_sats is None:
            target_sats = ('ISS (ZARYA)', "MOON")
        self.planets = load(planets_file)
        self.sat_TLEs = {}
        self.lines = urllib2.urlopen(url).read().decode('utf-8').replace('\r', '').split('\n')

        for sat in target_sats:
            if sat.lower() in self.planets:
                self.sat_TLEs[sat] = self.planets[sat.lower()]
                continue
            self.sat_TLEs[sat] = self._read_stations_file(sat)

        self.location = None
        self.update_location(*default_lat_long_alt)

    def _read_stations_file(self, sat):
        for i, line in enumerate(self.lines):
            if sat in line:
                TLE1 = self.lines[i+1]
                TLE2 = self.lines[i+2]
                return TLE1, TLE2

    def update_location(self, latitude, longitude, elevation: int):
        self.location = wgs84.latlon(latitude, longitude, elevation_m=elevation)
        return self.location

    def get_sat_altaz(self, sat):
        sat = sat.upper()

        if sat not in self.sat_TLEs:
            for s in self.sat_TLEs:
                if sat in s:
                    sat = s
                    break
        if sat not in self.sat_TLEs:
            return

        ts = load.timescale()
        t = ts.now()

        if sat in ("MOON", 'SUN', 'MARS'):
            mq = (self.planets['earth'] + self.location).at(t)
            m = mq.observe(self.sat_TLEs[sat]).apparent()
            # print(m.position.km)
            el, az, dist = m.altaz()
            return el.degrees, az.degrees, dist.km

        satellite = (EarthSatellite(*self.sat_TLEs[sat], sat, ts) - self.location).at(t)

        el, az, dist = satellite.altaz()
        return el.degrees, az.degrees, dist.km

    def get_latlong(self, sat):
        sat = sat.upper()

        if sat not in self.sat_TLEs:
            for s in self.sat_TLEs:
                if sat in s:
                    sat = s
                    break
        if sat not in self.sat_TLEs:
            return

        ts = load.timescale()
        t = ts.now()

        if sat in ("MOON", 'SUN', 'MARS'):
            earth = (self.planets['earth']).at(t)
            obj = earth.observe(self.sat_TLEs[sat]).apparent()
            lat, long, dist = obj.frame_latlon(itrs)
        else:
            satellite = (EarthSatellite(*self.sat_TLEs[sat], sat, ts)).at(t)
            lat, long, dist = satellite.frame_latlon(skyfield.framelib.itrs)

        lat, long, dist = lat.degrees, long.degrees, dist.km
        return lat, long, dist


if __name__ == "__main__":
    tle = TLEManager(target_sats=['MOON', 'ISS (ZARYA)', 'SUN', 'MARS'])

    while True:
        print( 'BODY    ELEVATION (DEG)      AZIMUTH (DEG)       DISTANCE (KM)')
        print(f'MOON: {tle.get_sat_altaz("MOON")}')
        print(f"ISS:  {tle.get_sat_altaz('ISS')}")
        print(f"SUN:  {tle.get_sat_altaz('SUN')}")
        print(f"MARS: {tle.get_sat_altaz('MARS')}")
        print()

        time.sleep(0.5)
