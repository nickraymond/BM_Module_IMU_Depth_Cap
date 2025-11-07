# /lib/imu_hw.py — Adafruit BNO055 hardware wrapper for your app
# CircuitPython 9.x
import time
import board, busio
try:
    import adafruit_bno055
except Exception:
    adafruit_bno055 = None

# --- Bring up I2C and BNO055 -------------------------------------------------
def init_bno(address=0x28, i2c=None, i2c_timeout_s=2.0):
    """
    Returns an initialized BNO055 object or None if unavailable.
    address: 0x28 (ADR->GND) or 0x29 (ADR->VCC)
    """
    if adafruit_bno055 is None:
        return None

    if i2c is None:
        # Try to lock I2C politely
        i2c = busio.I2C(board.SCL, board.SDA)
        t0 = time.monotonic()
        while not i2c.try_lock():
            if time.monotonic() - t0 > i2c_timeout_s:
                return None
            time.sleep(0.005)
        i2c.unlock()

    try:
        bno = adafruit_bno055.BNO055_I2C(i2c, address=address)
    except Exception:
        # Try alternate address automatically (helps field setups)
        if address == 0x28:
            try:
                bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
            except Exception:
                return None
        else:
            return None

    # Give sensor time to settle
    time.sleep(0.2)
    return bno

# --- Safe getters ------------------------------------------------------------
def get_quaternion(bno):
    """Return (w,x,y,z) or identity if sensor not ready."""
    if bno is None:
        return (1.0, 0.0, 0.0, 0.0)
    try:
        q = bno.quaternion  # tuple of 4 floats or None
        if not q:
            return (1.0, 0.0, 0.0, 0.0)
        # Some drivers return (None,None,...) briefly; guard it:
        if any(v is None for v in q):
            return (1.0, 0.0, 0.0, 0.0)
        return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
    except Exception:
        return (1.0, 0.0, 0.0, 0.0)

def get_cal_status(bno):
    """Return (sys, gyro, accel, mag) or (0,0,0,0) on error."""
    if bno is None:
        return (0, 0, 0, 0)
    try:
        return tuple(bno.calibration_status)
    except Exception:
        return (0, 0, 0, 0)

# --- Apply offsets to the device --------------------------------------------
def apply_offsets_from_cal_json(bno, cal_json: dict):
    """
    Map your JSON schema into BNO055 offset properties.
    JSON example:
      {"accel":{"bias":[...]},"gyro":{"bias":[...]},"mag":{"bias":[...]},...}
    """
    if bno is None or not cal_json:
        return False
    try:
        if "mag" in cal_json and "bias" in cal_json["mag"]:
            bno.offsets_magnetometer = tuple(cal_json["mag"]["bias"])
        if "gyro" in cal_json and "bias" in cal_json["gyro"]:
            bno.offsets_gyroscope = tuple(cal_json["gyro"]["bias"])
        if "accel" in cal_json and "bias" in cal_json["accel"]:
            bno.offsets_accelerometer = tuple(cal_json["accel"]["bias"])
        return True
    except Exception:
        return False
