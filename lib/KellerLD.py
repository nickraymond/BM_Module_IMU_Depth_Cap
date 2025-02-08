import time
import struct
import board
import busio
from adafruit_bus_device.i2c_device import I2CDevice

class KellerLD:
    _SLAVE_ADDRESS = 0x40
    _REQUEST_MEASUREMENT = 0xAC
    _DEBUG = False
    _P_MODES = (
        "PA Mode, Vented Gauge",   # Zero at atmospheric pressure
        "PR Mode, Sealed Gauge",   # Zero at 1.0 bar
        "PAA Mode, Absolute Gauge" # Zero at vacuum
    )
    _P_MODE_OFFSETS = (1.01325, 1.0, 0.0)

    def __init__(self, i2c_bus):
        self.i2c_device = I2CDevice(i2c_bus, self._SLAVE_ADDRESS)
        self.pMode = None
        self.pModeOffset = None
        self.pMin = None
        self.pMax = None

    def init(self):
        # Read pressure mode to determine the relevant offset
        with self.i2c_device:
            self.i2c_device.write(bytes([0x12]))
            time.sleep(0.001)
            data = bytearray(3)
            self.i2c_device.readinto(data)

        scaling0 = data[1] << 8 | data[2]
        self.debug(("0x12:", scaling0, data))

        pModeID = scaling0 & 0b11
        self.pMode = self._P_MODES[pModeID]
        self.pModeOffset = self._P_MODE_OFFSETS[pModeID]
        self.debug(("pMode", self.pMode, "pressure offset [bar]", self.pModeOffset))

        self.year = scaling0 >> 11
        self.month = (scaling0 & 0b0000011110000000) >> 7
        self.day = (scaling0 & 0b0000000001111100) >> 2
        self.debug(("calibration date", self.year, self.month, self.day))

        # Read out minimum pressure reading
        time.sleep(0.001)
        with self.i2c_device:
            self.i2c_device.write(bytes([0x13]))
            time.sleep(0.001)
            data = bytearray(3)
            self.i2c_device.readinto(data)
        MSWord = data[1] << 8 | data[2]
        self.debug(("0x13:", MSWord, data))

        time.sleep(0.001)
        with self.i2c_device:
            self.i2c_device.write(bytes([0x14]))
            time.sleep(0.001)
            data = bytearray(3)
            self.i2c_device.readinto(data)
        LSWord = data[1] << 8 | data[2]
        self.debug(("0x14:", LSWord, data))

        self.pMin = MSWord << 16 | LSWord
        self.debug(("pMin", self.pMin))

        # Read out maximum pressure reading
        time.sleep(0.001)
        with self.i2c_device:
            self.i2c_device.write(bytes([0x15]))
            time.sleep(0.001)
            data = bytearray(3)
            self.i2c_device.readinto(data)
        MSWord = data[1] << 8 | data[2]
        self.debug(("0x15:", MSWord, data))

        time.sleep(0.001)
        with self.i2c_device:
            self.i2c_device.write(bytes([0x16]))
            time.sleep(0.001)
            data = bytearray(3)
            self.i2c_device.readinto(data)
        LSWord = data[1] << 8 | data[2]
        self.debug(("0x16:", LSWord, data))

        self.pMax = MSWord << 16 | LSWord
        self.debug(("pMax", self.pMax))

        self.pMin = struct.unpack('f', struct.pack('I', self.pMin))[0]
        self.pMax = struct.unpack('f', struct.pack('I', self.pMax))[0]
        self.debug(("pMin:", self.pMin, "pMax:", self.pMax))

        return True

    def read(self):
        if self.pMin is None or self.pMax is None:
            print("Init required! Call init() first.")
            return False

        with self.i2c_device:
            self.i2c_device.write(bytes([self._REQUEST_MEASUREMENT]))

        time.sleep(0.01)  # 10 ms wait

        with self.i2c_device:
            data = bytearray(5)
            self.i2c_device.readinto(data)

        statusByte = data[0]
        pressureRaw = data[1] << 8 | data[2]
        temperatureRaw = data[3] << 8 | data[4]

        # Handle error cases based on statusByte
        if statusByte & 0b11 << 3:
            print(f"Invalid mode: {((statusByte & 0b11 << 3) >> 3)}")
            return False

        if statusByte & 1 << 2:
            print("Memory checksum error!")
            return False

        # Convert raw pressure and temperature values
        self._pressure = (pressureRaw - 16384) * (self.pMax - self.pMin) / 32768 + self.pMin + self.pModeOffset
        self._temperature = ((temperatureRaw >> 4) - 24) * 0.05 - 50

        self.debug(("data:", data))
        self.debug(("pressureRaw:", pressureRaw, "pressure:", self._pressure))
        self.debug(("temperatureRaw", temperatureRaw, "temperature:", self._temperature))

        return True

    def temperature(self):
        if self._temperature is None:
            print("Call read() first to get a measurement.")
            return
        return self._temperature

    def pressure(self):
        if self._pressure is None:
            print("Call read() first to get a measurement.")
            return
        return self._pressure

    def debug(self, msg):
        if self._DEBUG:
            print(msg)

    # Updated to work with Circuit Python
    def __str__(self):
        return ("Keller LD I2C Pressure/Temperature Transmitter\n"
                "\ttype: {}\n"
                "\tcalibration date: {}-{}-{}\n"
                "\tpressure offset: {:.5f} bar\n"
                "\tminimum pressure: {:.5f} bar\n"
                "\tmaximum pressure: {:.5f} bar").format(
                    self.pMode, self.year, self.month, self.day, 
                    self.pModeOffset, self.pMin, self.pMax)
