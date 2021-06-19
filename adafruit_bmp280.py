# SPDX-FileCopyrightText: 2017 ladyada for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_bmp280`
===============================================================================

CircuitPython driver from BMP280 Temperature and Barometric Pressure sensor

* Author(s): ladyada

Implementation Notes
--------------------

**Hardware:**

* `Adafruit from BMP280 Temperature and Barometric
  Pressure sensor <https://www.adafruit.com/product/2651>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""
import math
from time import sleep

try:
    import struct
except ImportError:
    import ustruct as struct
# from micropython import const
import pigpio

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BMP280.git"

#    I2C ADDRESS/BITS/SETTINGS
#    -----------------------------------------------------------------------
_CHIP_ID = 0x58

_REGISTER_CHIPID = 0xD0
_REGISTER_DIG_T1 = 0x88
_REGISTER_SOFTRESET = 0xE0
_REGISTER_STATUS = 0xF3
_REGISTER_CTRL_MEAS = 0xF4
_REGISTER_CONFIG = 0xF5
_REGISTER_PRESSUREDATA = 0xF7
_REGISTER_TEMPDATA = 0xFA


"""iir_filter values"""
IIR_FILTER_DISABLE = 0
IIR_FILTER_X2 = 0x01
IIR_FILTER_X4 = 0x02
IIR_FILTER_X8 = 0x03
IIR_FILTER_X16 = 0x04

_BMP280_IIR_FILTERS = (
    IIR_FILTER_DISABLE,
    IIR_FILTER_X2,
    IIR_FILTER_X4,
    IIR_FILTER_X8,
    IIR_FILTER_X16,
)

"""overscan values for temperature, pressure, and humidity"""
OVERSCAN_DISABLE = 0x00
OVERSCAN_X1 = 0x01
OVERSCAN_X2 = 0x02
OVERSCAN_X4 = 0x03
OVERSCAN_X8 = 0x04
OVERSCAN_X16 = 0x05

_BMP280_OVERSCANS = {
    OVERSCAN_DISABLE: 0,
    OVERSCAN_X1: 1,
    OVERSCAN_X2: 2,
    OVERSCAN_X4: 4,
    OVERSCAN_X8: 8,
    OVERSCAN_X16: 16,
}

"""mode values"""
MODE_SLEEP = 0x00
MODE_FORCE = 0x01
MODE_NORMAL = 0x03

_BMP280_MODES = (MODE_SLEEP, MODE_FORCE, MODE_NORMAL)
"""
standby timeconstant values
TC_X[_Y] where X=milliseconds and Y=tenths of a millisecond
"""
STANDBY_TC_0_5 = 0x00  # 0.5ms
STANDBY_TC_10 = 0x06  # 10ms
STANDBY_TC_20 = 0x07  # 20ms
STANDBY_TC_62_5 = 0x01  # 62.5ms
STANDBY_TC_125 = 0x02  # 125ms
STANDBY_TC_250 = 0x03  # 250ms
STANDBY_TC_500 = 0x04  # 500ms
STANDBY_TC_1000 = 0x05  # 1000ms

_BMP280_STANDBY_TCS = (
    STANDBY_TC_0_5,
    STANDBY_TC_10,
    STANDBY_TC_20,
    STANDBY_TC_62_5,
    STANDBY_TC_125,
    STANDBY_TC_250,
    STANDBY_TC_500,
    STANDBY_TC_1000,
)


class Adafruit_BMP280:  # pylint: disable=invalid-name
    """Base BMP280 object. Use :class:`Adafruit_BMP280_I2C` or :class:`Adafruit_BMP280_SPI`
    instead of this. This checks the BMP280 was found, reads the coefficients and
    enables the sensor for continuous reads

    .. note::
        The operational range of the BMP280 is 300-1100 hPa.
        Pressure measurements outside this range may not be as accurate.

    """

    def __init__(self):
        # Check device ID.
        chip_id = self._read_byte(_REGISTER_CHIPID)
        if _CHIP_ID != chip_id:
            raise RuntimeError("Failed to find BMP280! Chip ID 0x%x" % chip_id)
        # Set some reasonable defaults.
        self._iir_filter = IIR_FILTER_DISABLE
        self._overscan_temperature = OVERSCAN_X2
        self._overscan_pressure = OVERSCAN_X16
        self._t_standby = STANDBY_TC_0_5
        self._mode = MODE_SLEEP
        self._reset()
        self._read_coefficients()
        self._write_ctrl_meas()
        self._write_config()
        self.sea_level_pressure = 1013.25
        """Pressure in hectoPascals at sea level. Used to calibrate `altitude`."""
        self._t_fine = None

    def _read_temperature(self):
        # perform one measurement
        if self.mode != MODE_NORMAL:
            self.mode = MODE_FORCE
            # Wait for conversion to complete
            while self._get_status() & 0x08:
                sleep(0.002)
        raw_temperature = (
            self._read24(_REGISTER_TEMPDATA) / 16
        )  # lowest 4 bits get dropped
        # print("raw temp: ", UT)
        var1 = (
            raw_temperature / 16384.0 - self._temp_calib[0] / 1024.0
        ) * self._temp_calib[1]
        # print(var1)
        var2 = (
            (raw_temperature / 131072.0 - self._temp_calib[0] / 8192.0)
            * (raw_temperature / 131072.0 - self._temp_calib[0] / 8192.0)
        ) * self._temp_calib[2]
        # print(var2)

        self._t_fine = int(var1 + var2)
        # print("t_fine: ", self.t_fine)

    def _reset(self):
        """Soft reset the sensor"""
        self._write_register_byte(_REGISTER_SOFTRESET, 0xB6)
        sleep(0.004)  # Datasheet says 2ms.  Using 4ms just to be safe

    def _write_ctrl_meas(self):
        """
        Write the values to the ctrl_meas register in the device
        ctrl_meas sets the pressure and temperature data acquisition options
        """
        self._write_register_byte(_REGISTER_CTRL_MEAS, self._ctrl_meas)

    def _get_status(self):
        """Get the value from the status register in the device """
        return self._read_byte(_REGISTER_STATUS)

    def _read_config(self):
        """Read the value from the config register in the device """
        return self._read_byte(_REGISTER_CONFIG)

    def _write_config(self):
        """Write the value to the config register in the device """
        normal_flag = False
        if self._mode == MODE_NORMAL:
            # Writes to the config register may be ignored while in Normal mode
            normal_flag = True
            self.mode = MODE_SLEEP  # So we switch to Sleep mode first
        self._write_register_byte(_REGISTER_CONFIG, self._config)
        if normal_flag:
            self.mode = MODE_NORMAL

    @property
    def mode(self):
        """
        Operation mode
        Allowed values are set in the MODE enum class
        """
        return self._mode

    @mode.setter
    def mode(self, value):
        if not value in _BMP280_MODES:
            raise ValueError("Mode '%s' not supported" % (value))
        self._mode = value
        self._write_ctrl_meas()

    @property
    def standby_period(self):
        """
        Control the inactive period when in Normal mode
        Allowed standby periods are set the STANDBY enum class
        """
        return self._t_standby

    @standby_period.setter
    def standby_period(self, value):
        if not value in _BMP280_STANDBY_TCS:
            raise ValueError("Standby Period '%s' not supported" % (value))
        if self._t_standby == value:
            return
        self._t_standby = value
        self._write_config()

    @property
    def overscan_temperature(self):
        """
        Temperature Oversampling
        Allowed values are set in the OVERSCAN enum class
        """
        return self._overscan_temperature

    @overscan_temperature.setter
    def overscan_temperature(self, value):
        if not value in _BMP280_OVERSCANS:
            raise ValueError("Overscan value '%s' not supported" % (value))
        self._overscan_temperature = value
        self._write_ctrl_meas()

    @property
    def overscan_pressure(self):
        """
        Pressure Oversampling
        Allowed values are set in the OVERSCAN enum class
        """
        return self._overscan_pressure

    @overscan_pressure.setter
    def overscan_pressure(self, value):
        if not value in _BMP280_OVERSCANS:
            raise ValueError("Overscan value '%s' not supported" % (value))
        self._overscan_pressure = value
        self._write_ctrl_meas()

    @property
    def iir_filter(self):
        """
        Controls the time constant of the IIR filter
        Allowed values are set in the IIR_FILTER enum class
        """
        return self._iir_filter

    @iir_filter.setter
    def iir_filter(self, value):
        if not value in _BMP280_IIR_FILTERS:
            raise ValueError("IIR Filter '%s' not supported" % (value))
        self._iir_filter = value
        self._write_config()

    @property
    def _config(self):
        """Value to be written to the device's config register """
        config = 0
        if self.mode == MODE_NORMAL:
            config += self._t_standby << 5
        if self._iir_filter:
            config += self._iir_filter << 2
        return config

    @property
    def _ctrl_meas(self):
        """Value to be written to the device's ctrl_meas register """
        ctrl_meas = self.overscan_temperature << 5
        ctrl_meas += self.overscan_pressure << 2
        ctrl_meas += self.mode
        return ctrl_meas

    @property
    def measurement_time_typical(self):
        """Typical time in milliseconds required to complete a measurement in normal mode"""
        meas_time_ms = 1
        if self.overscan_temperature != OVERSCAN_DISABLE:
            meas_time_ms += 2 * _BMP280_OVERSCANS.get(self.overscan_temperature)
        if self.overscan_pressure != OVERSCAN_DISABLE:
            meas_time_ms += 2 * _BMP280_OVERSCANS.get(self.overscan_pressure) + 0.5
        return meas_time_ms

    @property
    def measurement_time_max(self):
        """Maximum time in milliseconds required to complete a measurement in normal mode"""
        meas_time_ms = 1.25
        if self.overscan_temperature != OVERSCAN_DISABLE:
            meas_time_ms += 2.3 * _BMP280_OVERSCANS.get(self.overscan_temperature)
        if self.overscan_pressure != OVERSCAN_DISABLE:
            meas_time_ms += 2.3 * _BMP280_OVERSCANS.get(self.overscan_pressure) + 0.575
        return meas_time_ms

    @property
    def temperature(self):
        """The compensated temperature in degrees Celsius."""
        self._read_temperature()
        return self._t_fine / 5120.0

    @property
    def pressure(self):
        """
        The compensated pressure in hectoPascals.
        returns `None` if pressure measurement is disabled
        """
        self._read_temperature()

        # Algorithm from the BMP280 driver
        # https://github.com/BoschSensortec/BMP280_driver/blob/master/bmp280.c
        adc = self._read24(_REGISTER_PRESSUREDATA) / 16  # lowest 4 bits get dropped
        var1 = float(self._t_fine) / 2.0 - 64000.0
        var2 = var1 * var1 * self._pressure_calib[5] / 32768.0
        var2 = var2 + var1 * self._pressure_calib[4] * 2.0
        var2 = var2 / 4.0 + self._pressure_calib[3] * 65536.0
        var3 = self._pressure_calib[2] * var1 * var1 / 524288.0
        var1 = (var3 + self._pressure_calib[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._pressure_calib[0]
        if not var1:  # avoid exception caused by division by zero
            raise ArithmeticError(
                "Invalid result possibly related to error while reading the calibration registers"
            )
        pressure = 1048576.0 - adc
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = self._pressure_calib[8] * pressure * pressure / 2147483648.0
        var2 = pressure * self._pressure_calib[7] / 32768.0
        pressure = pressure + (var1 + var2 + self._pressure_calib[6]) / 16.0
        pressure /= 100

        return pressure

    @property
    def altitude(self):
        """The altitude based on the sea level pressure (:attr:`sea_level_pressure`)
        - which you must enter ahead of time)"""
        p = self.pressure  # in Si units for hPascal
        return 44330 * (1.0 - math.pow(p / self.sea_level_pressure, 0.1903))

    ####################### Internal helpers ################################
    def _read_coefficients(self):
        """Read & save the calibration coefficients"""
        coeff = self._read_register(_REGISTER_DIG_T1, 24)
        coeff = list(struct.unpack("<HhhHhhhhhhhh", bytes(coeff)))
        coeff = [float(i) for i in coeff]
        # The temp_calib lines up with DIG_T# registers.
        self._temp_calib = coeff[:3]
        self._pressure_calib = coeff[3:]
        # print("%d %d %d" % (self._temp_calib[0], self._temp_calib[1], self._temp_calib[2]))
        # print("%d %d %d" % (self._pressure_calib[0], self._pressure_calib[1],
        #                     self._pressure_calib[2]))
        # print("%d %d %d" % (self._pressure_calib[3], self._pressure_calib[4],
        #                     self._pressure_calib[5]))
        # print("%d %d %d" % (self._pressure_calib[6], self._pressure_calib[7],
        #                     self._pressure_calib[8]))

    def _read_byte(self, register):
        """Read a byte register value and return it"""
        return self._read_register(register, 1)[0]

    def _read24(self, register):
        """Read an unsigned 24-bit value as a floating point and return it."""
        ret = 0.0
        for b in self._read_register(register, 3):
            ret *= 256.0
            ret += float(b & 0xFF)
        return ret

    def _read_register(self, register, length):
        """Low level register reading, not implemented in base class"""
        raise NotImplementedError()

    def _write_register_byte(self, register, value):
        """Low level register writing, not implemented in base class"""
        raise NotImplementedError()


class Adafruit_BMP280_PIGPIO_I2C(Adafruit_BMP280):  # pylint: disable=invalid-name
    """Driver for I2C connected BMP280.

    :param ~busio.I2C i2c: The I2C bus the BMP280 is connected to.
    :param int address: I2C device address. Defaults to :const:`0x77`.
                        but another address can be passed in as an argument

    **Quickstart: Importing and using the BMP280**

        Here is an example of using the :class:`BMP280_I2C` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_bmp280

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()   # uses board.SCL and board.SDA
            bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)

        You need to setup the pressure at sea level

        .. code-block:: python

            bmp280.sea_level_pressure = 1013.25

        Now you have access to the :attr:`temperature`,
        :attr:`pressure` and :attr:`altitude` attributes

        .. code-block:: python

            temperature = bmp280.temperature
            pressure = bmp280.pressure
            altitude = bmp280.altitude

    """

    def __init__(self, pi:pigpio.pi, bus=1, address=0x77):
        self._pi = pi
        self._i2c = pi.i2c_open(bus, address)
        super().__init__()

    def _read_register(self, register, length):
        """Low level register reading over I2C, returns a list of values"""
        _, result = self._pi.i2c_read_i2c_block_data(self._i2c, register, length)
        return result

    def _write_register_byte(self, register, value):
        """Low level register writing over I2C, writes one 8-bit value"""
        self._pi.i2c_write_byte_data(self._i2c, register, value)



if __name__ == "__main__":
    pi = pigpio.pi('192.168.178.229')
    if pi:
        imu = Adafruit_BMP280_PIGPIO_I2C(pi)
        while True:
            print(imu.altitude)
