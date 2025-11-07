# /lib/imu_cal_manager.py — exact Adafruit-style blocking calibrator
import time, json

class Mode:
    NDOF_MODE = 0x0C

class IMUCalManager:
    def __init__(self, *, bno, cal_json_path, read_json, write_json_atomic,
                 cal_defaults, ack_fn, led_success_fn, led_working_fn, led_error_fn):
        self.bno = bno
        self.cal_json_path = cal_json_path
        self.read_json = read_json
        self.write_json_atomic = write_json_atomic
        self.cal_defaults = cal_defaults
        self.ack = ack_fn
        self.led_ok = led_success_fn
        self.led_work = led_working_fn
        self.led_err = led_error_fn

    def run_blocking_exact(self):
        """Exact flow as Adafruit example:
           - set NDOF
           - wait MAG->ACCEL->GYRO to hit 3
           - then read offsets and save/apply
        """
        if self.bno is None:
            self.ack("CAL ERR: no IMU")
            self.led_err()
            return False

        # Set mode exactly, then let it settle
        try:
            self.bno.mode = Mode.NDOF_MODE
        except Exception:
            pass
        time.sleep(1.0)

        # MAG step
        self.ack("Magnetometer: figure-eight until status=3")
        while True:
            try:
                m = int(self.bno.calibration_status[3])
            except Exception:
                m = 0
            self.ack("Mag Calib: {:3.0f}%".format(m * 100 / 3))
            self.led_work()
            if m == 3:
                self.ack("Mag: CALIBRATED")
                self.led_ok()
                break
            time.sleep(1)

        # ACCEL step
        self.ack("Accelerometer: six faces until status=3")
        while True:
            try:
                a = int(self.bno.calibration_status[2])
            except Exception:
                a = 0
            self.ack("Accel Calib: {:3.0f}%".format(a * 100 / 3))
            self.led_work()
            if a == 3:
                self.ack("Accel: CALIBRATED")
                self.led_ok()
                break
            time.sleep(1)

        # GYRO step
        self.ack("Gyroscope: hold still until status=3")
        while True:
            try:
                g = int(self.bno.calibration_status[1])
            except Exception:
                g = 0
            self.ack("Gyro Calib: {:3.0f}%".format(g * 100 / 3))
            self.led_work()
            if g == 3:
                self.ack("Gyro: CALIBRATED")
                self.led_ok()
                break
            time.sleep(1)

        # Read offsets once all 3 are done
        try:
            off_mag   = tuple(self.bno.offsets_magnetometer)
            off_gyro  = tuple(self.bno.offsets_gyroscope)
            off_accel = tuple(self.bno.offsets_accelerometer)
        except Exception:
            self.ack("CAL ERR: could not read offsets")
            self.led_err()
            return False

        cal = self.read_json(self.cal_json_path, self.cal_defaults)
        cal["mag"]["bias"]   = list(off_mag)
        cal["gyro"]["bias"]  = list(off_gyro)
        cal["accel"]["bias"] = list(off_accel)

        if not self.write_json_atomic(self.cal_json_path, cal):
            self.ack("CAL ERR: save failed")
            self.led_err()
            return False

        # Try to apply back to device (driver handles CONFIG mode internally)
        try:
            self.bno.offsets_magnetometer  = tuple(cal["mag"]["bias"])
            self.bno.offsets_gyroscope     = tuple(cal["gyro"]["bias"])
            self.bno.offsets_accelerometer = tuple(cal["accel"]["bias"])
        except Exception:
            self.ack("CAL WARN: could not apply to device (non-fatal)")

        self.ack("CAL SAVED: " + json.dumps(cal))
        self.led_ok()
        return True
