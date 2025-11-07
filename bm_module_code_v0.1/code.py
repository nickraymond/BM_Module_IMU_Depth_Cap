# code.py — MVP logger + BM ACKs everywhere + LED color control
# - Always ACKs each BM command to spotter/printf
# - LED control via device/led with color
# - Uses one config file: /config/system.json
# - Graceful sensor failures (prints NA)
# - Keeps your original raw & summary behavior

# Set raw period:
# bm pub device/sample_period/set {"sample_period_s":1.0} text 0
#
# Set summary period:
# bm pub device/summary_period/set {"summary_period_s":60} text 0
#
# Get sample and aggregation period
# bm pub device/config/get {} text 0
#
# IMU calibration (blocking):
# bm pub imu/calibration/start {} text 0
#
# Quaternion offset:
#
# Zero to current pose:
#     bm pub imu/quaternion/zero {} text 0
#
# Set to target RPY degrees:
#     bm pub imu/quaternion/target {"target_deg":[45,35,0]} text 0
#
# Inspect:
#     bm pub imu/quaternion/get_offset {} text 0
#
# Force set:
#     bm pub imu/quaternion/set_offset {"q":[1,0,0,0]} text 0
#
# Sensor tests (one-shot prints):
#     bm pub sensor/depth/test {} text 0
#     bm pub sensor/imu/test {} text 0
#     bm pub sensor/cap/test {} text 0
#     bm pub sensor/all/test {} text 0
#     bm pub sensor/all/offsets {} text 0

import time, os, json, math
import board, neopixel

from imu_module import IMU
from KellerLD import KellerLD
from capacitive import CapacitiveSensor
from stats import SensorStats
from bm_serial import BristlemouthSerial

# ----------- boot diagnostic ---------
def fs_is_writable() -> bool:
    try:
        with open("/.__fs_probe__", "w") as f:
            f.write("x")
        os.remove("/.__fs_probe__")
        return True
    except Exception:
        return False


# ---------------- LED ----------------
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.brightness = 0.35
LED_COLORS = {
    "success": (0,255,0),
    "error": (255,0,0),
    "working": (0,0,255),
    "transmitting": (0,255,255),
    "white": (255,255,255),
    "off": (0,0,0),
}
_latched = LED_COLORS["off"]

def _clip(c): return max(0, min(255, int(c)))
def _set_led(rgb):
    global _latched
    pixel[0] = rgb
    _latched = rgb

def led_color_by_name(name):
    if not name: return LED_COLORS["white"]
    n = str(name).lower()
    if n in LED_COLORS: return LED_COLORS[n]
    # hex #RRGGBB?
    if n.startswith("#") and len(n) == 7:
        try:
            r = int(n[1:3], 16); g = int(n[3:5], 16); b = int(n[5:7], 16)
            return (_clip(r), _clip(g), _clip(b))
        except Exception:
            pass
    # comma "r,g,b"?
    if "," in n:
        try:
            r,g,b = [int(x) for x in n.split(",")[:3]]
            return (_clip(r), _clip(g), _clip(b))
        except Exception:
            pass
    return LED_COLORS["white"]

def led_blink(rgb, on_ms=150, off_ms=150, count=3):
    restore = _latched
    for _ in range(max(1,int(count))):
        pixel[0] = rgb; time.sleep(on_ms/1000.0)
        pixel[0] = restore; time.sleep(off_ms/1000.0)

def led_success(): led_blink(LED_COLORS["success"], 60, 40, 2)

def led_working():
    c = _latched; pixel[0] = LED_COLORS["working"]; time.sleep(0.02); pixel[0] = c

def led_error():   led_blink(LED_COLORS["error"], 90, 50, 2)

# --- Non-blocking TX pulse state ---
_tx_until_ms = 0.0
_tx_restore  = LED_COLORS["off"]

def led_tx_pulse(duration_ms=300):
    """Show 'transmitting' color briefly, then auto-restore previous LED."""
    global _tx_until_ms, _tx_restore
    _tx_restore = _latched
    _set_led(LED_COLORS["transmitting"])
    _tx_until_ms = time.monotonic() * 1000.0 + float(duration_ms)


# --------------- Config --------------
CAL_ACTIVE = False  # when True, skip normal sensor reads
CONFIG_DIR = "/config"
SYSTEM_JSON_PATH = "/config/system.json"
SYSTEM_DEFAULTS = {
    "sample_period_s": 1.0,
    "summary_period_s": 300.0,
    "keller": {"pressure_bias_dbar": 0.0, "temp_bias_c": 0.0},
    "capacitive": {"offset": 0.0, "scale": 1.0, "water_thresh": 0.5},
    "orientation": {"q_offset": [1.0,0.0,0.0,0.0]},
    "imu": {
        "accel": {"bias":[0.0,0.0,0.0]},
        "gyro":  {"bias":[0.0,0.0,0.0]},
        "mag":   {"bias":[0.0,0.0,0.0]}
    }
}

def ensure_dir(path):
    try: os.listdir(path)
    except Exception:
        try: os.mkdir(path)
        except Exception: pass

# Try your bm_store if available
try:
    from bm_store import read_json as _rj, write_json_atomic as _wja, ensure_dir as _ed
    def read_json(p, d): return _rj(p, d)
    def write_json_atomic(p, o): return _wja(p, o)
    ensure_dir = _ed
except Exception:
    def read_json(path, dflt):
        try:
            with open(path, "r") as f: return json.load(f)
        except Exception:
            return dflt
    def write_json_atomic(path, obj):
        tmp = path + ".tmp"
        try:
            with open(tmp, "w") as f: json.dump(obj, f)
            os.rename(tmp, path); return True
        except Exception:
            try: os.remove(tmp)
            except Exception: pass
            return False

# --------------- BM topics -----------
TOP_SAMPLE_PERIOD_SET  = "device/sample_period/set"   # {"sample_period_s": 1.0}
TOP_SUMMARY_PERIOD_SET = "device/summary_period/set"  # {"summary_period_s": 300}

TOP_CFG_GET = "device/config/get"
TOP_CFG_SET = "device/config/set"

TOP_CAL_START   = "imu/calibration/start"
TOP_Q_ZERO      = "imu/quaternion/zero"
TOP_Q_TARGET    = "imu/quaternion/target"      # {"target_deg":[r,p,y]}
TOP_Q_GET       = "imu/quaternion/get_offset"
TOP_Q_SET       = "imu/quaternion/set_offset"  # {"q":[w,x,y,z]}

TOP_LED         = "device/led"                 # {"led":"on|off|blink","color":"white","#00ffaa","r,g,b","period_ms":500,"count":3}

TOP_TEST_DEPTH  = "sensor/depth/test"
TOP_TEST_IMU    = "sensor/imu/test"
TOP_TEST_CAP    = "sensor/cap/test"
TOP_TEST_ALL    = "sensor/all/test"
TOP_OFFSETS_ALL = "sensor/all/offsets"

# -------------- Helpers --------------
def ack(bm, msg):
    try: bm.spotter_print(msg)
    except Exception: pass
    print("[ACK]", msg)

def ack_rx(bm, topic):  # received confirmation for every command
    ack(bm, "RX: {}".format(topic))

def parse_json(text):
    try: return json.loads(text.strip().rstrip("\x00\r\n"))
    except Exception: return None

def is_num(x):
    try: return (x is not None) and (not isinstance(x,bool)) and math.isfinite(float(x))
    except Exception: return False

def same_topic(rx, expect):
    if rx is None: return False
    return rx.rstrip("\x00 \r\n") == expect

def safe_text(b):
    try: return b.decode("utf-8")
    except Exception:
        try: return str(b, "utf-8")
        except Exception: return None

# -------------- Bring up HW -----------
i2c = board.I2C()

ensure_dir(CONFIG_DIR)
cfg = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
write_json_atomic(SYSTEM_JSON_PATH, cfg)  # ensure file exists

sample_period_s  = float(cfg.get("sample_period_s", 1.0))
summary_period_s = float(cfg.get("summary_period_s", 300.0))
SENSOR_POLL_INTERVAL_MS = max(50, int(sample_period_s * 1000))
AGGREGATION_INTERVAL_MS = max(1000, int(summary_period_s * 1000))

kel_bias_p = float(((cfg.get("keller") or {}).get("pressure_bias_dbar") or 0.0))
kel_bias_t = float(((cfg.get("keller") or {}).get("temp_bias_c") or 0.0))
cap_off    = float(((cfg.get("capacitive") or {}).get("offset") or 0.0))
cap_scl    = float(((cfg.get("capacitive") or {}).get("scale") or 1.0))

q_offset = tuple((cfg.get("orientation") or {}).get("q_offset", [1.0,0.0,0.0,0.0]))

imu = None;  imu_ok = False
keller = None; keller_ok = False
capacitive = None; cap_ok = False

try:
    imu = IMU(i2c)
    if hasattr(imu, "set_quaternion_offset"):
        try: imu.set_quaternion_offset(q_offset)
        except Exception: pass
    imu_ok = True
except Exception as e:
    print("IMU init FAILED:", e)

try:
    keller = KellerLD(i2c)
    if keller.init(): keller_ok = True
    else: print("Keller init FAILED: init() returned False")
except Exception as e:
    print("Keller init FAILED:", e)

try:
    capacitive = CapacitiveSensor(board.A0)
    cap_ok = True
except Exception as e:
    print("Capacitive init FAILED:", e)

# -------------- Stats -----------------
stats = {
    "pressure":    SensorStats(),
    "temperature": SensorStats(),
    "capacitive":  SensorStats(),
    "roll":        SensorStats(),
    "pitch":       SensorStats(),
    "yaw":         SensorStats(),
    "tilt":        SensorStats(),
}

# -------------- BM bus ----------------
bm = BristlemouthSerial()

# ---- IMU calibration via imu_cal_manager (optional)
def handle_calibration_start():
    # IMPORTANT: access the same IMU instance used by the main loop
    global imu, imu_ok, CAL_ACTIVE
    if not imu_ok or imu is None:
        ack(bm, "CAL ERR: no IMU")
        return

    ack(bm, "CAL: begin (exact)")
    CAL_ACTIVE = True
    try:
        mag_off, gyr_off, acc_off = calibrate_bno_exact(imu)
        ack(bm, f"CAL SAVED: mag={mag_off} gyro={gyr_off} accel={acc_off}")
    except Exception as e:
        ack(bm, f"CAL ERR: {e}")
    finally:
        CAL_ACTIVE = False


# ---- Quaternion helpers
def euler_to_q(r,p,y):
    r*=math.pi/180.0; p*=math.pi/180.0; y*=math.pi/180.0
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (cy*cp*cr + sy*sp*sr,
            cy*cp*sr - sy*sp*cr,
            sy*cp*sr + cy*sp*cr,
            sy*cp*cr - cy*sp*sr)

def q_conj(q): return (q[0], -q[1], -q[2], -q[3])
def q_mul(a,b):
    aw,ax,ay,az = a; bw,bx,by,bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def get_q_now():
    if not imu_ok: return (1.0,0.0,0.0,0.0)
    if hasattr(imu, "get_quaternion"):
        try:
            q = imu.get_quaternion()
            if q and all(v is not None for v in q):
                return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        except Exception:
            pass
    return (1.0,0.0,0.0,0.0)

def handle_q_zero():
    if not imu_ok: ack(bm, "ORIENT ERR: IMU missing"); return
    q = get_q_now()
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    cfg2.setdefault("orientation", {})["q_offset"] = [float(v) for v in q]
    if write_json_atomic(SYSTEM_JSON_PATH, cfg2):
        if hasattr(imu, "set_quaternion_offset"):
            try: imu.set_quaternion_offset(tuple(cfg2["orientation"]["q_offset"]))
            except Exception: pass
        ack(bm, "ORIENT: zeroed & saved")
    else:
        ack(bm, "ORIENT ERR: save failed")

def handle_q_target(js):
    if not imu_ok: ack(bm, "ORIENT ERR: IMU missing"); return
    tgt = js.get("target_deg") or js.get("target")
    if not (isinstance(tgt, list) and len(tgt)==3):
        ack(bm, "ORIENT ERR: expected target_deg [r,p,y]"); return
    r,p,y = float(tgt[0]), float(tgt[1]), float(tgt[2])
    q_offset = q_mul(get_q_now(), q_conj(euler_to_q(r,p,y)))
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    cfg2.setdefault("orientation", {})["q_offset"] = [float(v) for v in q_offset]
    if write_json_atomic(SYSTEM_JSON_PATH, cfg2):
        if hasattr(imu, "set_quaternion_offset"):
            try: imu.set_quaternion_offset(tuple(cfg2["orientation"]["q_offset"]))
            except Exception: pass
        ack(bm, "ORIENT: target set & saved")
    else:
        ack(bm, "ORIENT ERR: save failed")

def handle_q_get():
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    q = (cfg2.get("orientation") or {}).get("q_offset", [1,0,0,0])
    ack(bm, "ORIENT: q_offset " + json.dumps(q))

def handle_q_set(js):
    q = js.get("q")
    if not (isinstance(q, list) and len(q)==4):
        ack(bm, "ORIENT ERR: expected {\"q\":[w,x,y,z]}"); return
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    cfg2.setdefault("orientation", {})["q_offset"] = [float(q[0]),float(q[1]),float(q[2]),float(q[3])]
    if write_json_atomic(SYSTEM_JSON_PATH, cfg2):
        if hasattr(imu, "set_quaternion_offset"):
            try: imu.set_quaternion_offset(tuple(cfg2["orientation"]["q_offset"]))
            except Exception: pass
        ack(bm, "ORIENT: q_offset set & saved")
    else:
        ack(bm, "ORIENT ERR: save failed")


# --- Drop-in probe: compare BNO Euler vs our quaternion-corrected Euler ---
def _q_norm(q):
    w,x,y,z = q
    n = (w*w + x*x + y*y + z*z) ** 0.5
    return (w/n, x/n, y/n, z/n) if n else (1.0,0.0,0.0,0.0)

def _q_conj(q): return (q[0], -q[1], -q[2], -q[3])

def _q_mul(a,b):
    aw,ax,ay,az = a; bw,bx,by,bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)

def _q_to_euler_rpy(q):
    # roll (x), pitch (y), yaw (z)
    w,x,y,z = q
    # roll
    sr = 2*(w*x + y*z); cr = 1 - 2*(x*x + y*y)
    roll = math.degrees(math.atan2(sr, cr))
    # pitch
    sp = 2*(w*y - z*x)
    pitch = math.degrees(math.copysign(math.pi/2, sp)) if abs(sp) >= 1 else math.degrees(math.asin(sp))
    # yaw
    sy = 2*(w*z + x*y); cy = 1 - 2*(y*y + z*z)
    yaw = math.degrees(math.atan2(sy, cy))
    return (roll, pitch, yaw)

def debug_dump_once():
    # 1) what IMU.get_corrected_orientation() currently returns
    r1=p1=y1=t1 = (None,)*4
    try:
        r1,p1,y1,t1 = imu.get_corrected_orientation()
    except Exception:
        pass

    # 2) get raw quaternion from IMU (you likely have imu.get_quaternion())
    q_now = (1.0,0.0,0.0,0.0)
    try:
        q_now = imu.get_quaternion()  # (w,x,y,z) from your driver
    except Exception:
        pass
    q_now = _q_norm(q_now)

    # 3) q_offset loaded from system.json (same one your code uses)
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    q_off_list = (cfg2.get("orientation") or {}).get("q_offset", [1,0,0,0])
    q_off = _q_norm((float(q_off_list[0]), float(q_off_list[1]), float(q_off_list[2]), float(q_off_list[3])))

    # Two candidate ways offsets are applied in various stacks:
    # A) corrected = conj(q_off) * q_now
    qA = _q_mul(_q_conj(q_off), q_now)
    rA,pA,yA = _q_to_euler_rpy(_q_norm(qA))

    # B) corrected = q_off * q_now
    qB = _q_mul(q_off, q_now)
    rB,pB,yB = _q_to_euler_rpy(_q_norm(qB))

    print("[IMU.get_corrected_orientation] R={:.1f} P={:.1f} Y={:.1f}".format(r1 or 0, p1 or 0, y1 or 0))
    print("[Qpath A: conj(off)*now]        R={:.1f} P={:.1f} Y={:.1f}".format(rA, pA, yA))
    print("[Qpath B: off*now]              R={:.1f} P={:.1f} Y={:.1f}".format(rB, pB, yB))


def calibrate_bno_exact(imu_obj):
    """
    Blocks until BNO055 is calibrated (mag, accel, gyro).
    Uses Adafruit driver via imu_obj.bno, then writes tuples into imu_obj and system.json.
    """
    bno = imu_obj.bno
    # ensure fusion mode that uses mag (absolute yaw)
    try:
        bno.mode = 0x0C  # NDOF
    except Exception:
        pass

    # Step 1: magnetometer (figure-eight)
    while True:
        try:
            sys, g, a, m = bno.calibration_status  # (sys, gyro, accel, mag)
        except Exception:
            raise RuntimeError("cal status read failed")
        ack(bm, f"CAL mag={m}/3 accel={a}/3 gyro={g}/3")  # lightweight progress
        if m >= 3:
            break
        time.sleep(0.5)

    # Step 2: accelerometer (six positions)
    while True:
        sys, g, a, m = bno.calibration_status
        ack(bm, f"CAL mag={m}/3 accel={a}/3 gyro={g}/3")
        if a >= 3:
            break
        time.sleep(0.5)

    # Step 3: gyro (hold still)
    while True:
        sys, g, a, m = bno.calibration_status
        ack(bm, f"CAL mag={m}/3 accel={a}/3 gyro={g}/3")
        if g >= 3:
            break
        time.sleep(0.5)

    # Read back tuples from the sensor
    mag_off  = bno.offsets_magnetometer
    gyr_off  = bno.offsets_gyroscope
    acc_off  = bno.offsets_accelerometer

    # Update IMU object for immediate use
    imu_obj.calibration_offsets = {
        "magnetometer":   mag_off,
        "gyroscope":      gyr_off,
        "accelerometer":  acc_off,
    }
    imu_obj.apply_calibration_offsets()

    # Persist to system.json
    cfg = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    cfg.setdefault("imu", {})
    cfg["imu"]["calibration_offsets"] = {
        "magnetometer":   list(mag_off),
        "gyroscope":      list(gyr_off),
        "accelerometer":  list(acc_off),
    }
    if not write_json_atomic(SYSTEM_JSON_PATH, cfg):
        raise RuntimeError("persist failed")

    return (mag_off, gyr_off, acc_off)




# ---- Timing setters
def handle_sample_period_set(js):
    sp = js.get("sample_period_s")
    ack(bm, "RX: device/sample_period/set")  # explicit receive ACK
    if not is_num(sp) or sp <= 0:
        ack(bm, "CFG ERR: invalid sample_period_s"); return
    global SENSOR_POLL_INTERVAL_MS, sample_period_s
    sample_period_s = float(sp)
    SENSOR_POLL_INTERVAL_MS = max(50, int(sample_period_s*1000))
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    cfg2["sample_period_s"] = sample_period_s
    if write_json_atomic(SYSTEM_JSON_PATH, cfg2):
        ack(bm, "CFG SAVED: sample_period_s={} ({} ms)".format(sample_period_s, SENSOR_POLL_INTERVAL_MS))
    else:
        ack(bm, "CFG ERR: write failed; using sample_period_s={} ({} ms)".format(sample_period_s, SENSOR_POLL_INTERVAL_MS))

def handle_summary_period_set(js):
    sp = js.get("summary_period_s")
    ack(bm, "RX: device/summary_period/set")
    if not is_num(sp) or sp <= 0:
        ack(bm, "CFG ERR: invalid summary_period_s"); return
    global AGGREGATION_INTERVAL_MS, summary_period_s
    summary_period_s = float(sp)
    AGGREGATION_INTERVAL_MS = max(1000, int(summary_period_s*1000))
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    cfg2["summary_period_s"] = summary_period_s
    if write_json_atomic(SYSTEM_JSON_PATH, cfg2):
        ack(bm, "CFG SAVED: summary_period_s={} ({} ms)".format(summary_period_s, AGGREGATION_INTERVAL_MS))
    else:
        ack(bm, "CFG ERR: write failed; using summary_period_s={} ({} ms)".format(summary_period_s, AGGREGATION_INTERVAL_MS))

# ---- Generic config get/set
def handle_cfg_get():
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    ack(bm, "CFG: " + json.dumps(cfg2))

def handle_cfg_set(js):
    ack(bm, "RX: device/config/set")
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    if "sample_period_s" in js and is_num(js["sample_period_s"]) and js["sample_period_s"]>0:
        cfg2["sample_period_s"] = float(js["sample_period_s"])
    if "summary_period_s" in js and is_num(js["summary_period_s"]) and js["summary_period_s"]>0:
        cfg2["summary_period_s"] = float(js["summary_period_s"])
    if "orientation" in js and isinstance(js["orientation"], dict):
        if "q_offset" in js["orientation"] and isinstance(js["orientation"]["q_offset"], list):
            cfg2.setdefault("orientation", {})["q_offset"] = js["orientation"]["q_offset"]
    if "imu" in js and isinstance(js["imu"], dict):
        for sec in ("accel","gyro","mag"):
            if sec in js["imu"] and "bias" in js["imu"][sec]:
                cfg2.setdefault("imu", {}).setdefault(sec, {})["bias"] = js["imu"][sec]["bias"]
    if "keller" in js and isinstance(js["keller"], dict):
        for k in ("pressure_bias_dbar","temp_bias_c"):
            if k in js["keller"] and is_num(js["keller"][k]):
                cfg2.setdefault("keller", {})[k] = float(js["keller"][k])
    if "capacitive" in js and isinstance(js["capacitive"], dict):
        for k in ("offset","scale","water_thresh"):
            if k in js["capacitive"] and is_num(js["capacitive"][k]):
                cfg2.setdefault("capacitive", {})[k] = float(js["capacitive"][k])

    if write_json_atomic(SYSTEM_JSON_PATH, cfg2):
        ack(bm, "CFG SAVED: " + json.dumps(cfg2))
        # apply updated periods immediately
        handle_sample_period_set({"sample_period_s": cfg2["sample_period_s"]})
        handle_summary_period_set({"summary_period_s": cfg2["summary_period_s"]})
    else:
        ack(bm, "CFG ERR: write failed")

# ---- LED control
def handle_led(js):
    ack(bm, "RX: device/led")
    mode = (js.get("led") or "").lower()
    color = led_color_by_name(js.get("color", "white"))
    if mode == "on":
        _set_led(color); ack(bm, "LED ACK: on {}".format(js.get("color","white"))); return
    if mode == "off":
        _set_led(LED_COLORS["off"]); ack(bm, "LED ACK: off"); return
    if mode == "blink":
        period_ms = int(js.get("period_ms", 500))
        count = int(js.get("count", 3))
        on_ms = max(1, period_ms//2); off_ms = max(0, period_ms - on_ms)
        led_blink(color, on_ms, off_ms, count)
        ack(bm, "LED ACK: blink color={} period_ms={} count={}".format(js.get("color","white"), period_ms, count))
        return
    ack(bm, "LED ERR: unknown")

# ---- Sensor tests
def test_depth():
    if not keller_ok: ack(bm, "KELLER: missing"); return
    if not keller.read(): ack(bm, "KELLER: read failed"); return
    try:
        p = float(keller.pressure()) + kel_bias_p
        t = float(keller.temperature()) + kel_bias_t
        ack(bm, "KELLER: p_dbar={:.3f}, t_c={:.2f}".format(p, t))
    except Exception:
        ack(bm, "KELLER: invalid readings")

def test_imu():
    if not imu_ok: ack(bm, "IMU: missing"); return
    try:
        r,p,y,tilt = imu.get_corrected_orientation()
        ack(bm, "IMU: r={:.2f}, p={:.2f}, y={:.2f}, tilt={:.2f}".format(r,p,y,tilt))
    except Exception:
        ack(bm, "IMU: read failed")

def test_cap():
    if not cap_ok: ack(bm, "CAP: missing"); return
    try:
        raw = float(capacitive.read_value())
        cal = cap_off + cap_scl*raw
        ack(bm, "CAP: raw={:.3f}, cal={:.3f}".format(raw, cal))
    except Exception:
        ack(bm, "CAP: read failed")

def test_all():
    test_depth(); test_imu(); test_cap()

def report_all_offsets():
    cfg2 = read_json(SYSTEM_JSON_PATH, SYSTEM_DEFAULTS)
    msg = {
        "orientation": {"q_offset": (cfg2.get("orientation") or {}).get("q_offset", [1,0,0,0])},
        "imu": {
            "accel": {"bias": ((cfg2.get("imu") or {}).get("accel") or {}).get("bias",[0,0,0])},
            "gyro":  {"bias": ((cfg2.get("imu") or {}).get("gyro")  or {}).get("bias",[0,0,0])},
            "mag":   {"bias": ((cfg2.get("imu") or {}).get("mag")   or {}).get("bias",[0,0,0])},
        },
        "keller": {
            "pressure_bias_dbar": ((cfg2.get("keller") or {}).get("pressure_bias_dbar", 0.0)),
            "temp_bias_c":        ((cfg2.get("keller") or {}).get("temp_bias_c", 0.0))
        },
        "capacitive": {
            "offset": ((cfg2.get("capacitive") or {}).get("offset", 0.0)),
            "scale":  ((cfg2.get("capacitive") or {}).get("scale", 1.0)),
            "water_thresh": ((cfg2.get("capacitive") or {}).get("water_thresh", 0.5))
        }
    }
    ack(bm, "OFFSETS: " + json.dumps(msg))

# ---- BM router (ALWAYS ACKS receipt)
def tap_router(node_id, msg_type, version, topic_len, topic, data_len, data):
    text = safe_text(data)
    js = parse_json(text) if text else None

    # Always ACK that the topic was received
    ack_rx(bm, topic if isinstance(topic, str) else str(topic))

    # Validate JSON for handlers that expect it
    needs_json = (
        TOP_SAMPLE_PERIOD_SET, TOP_SUMMARY_PERIOD_SET,
        TOP_CFG_SET, TOP_Q_TARGET, TOP_Q_SET, TOP_LED
    )
    if any(same_topic(topic, t) for t in needs_json) and js is None:
        ack(bm, "ERR: bad JSON for topic {}".format(topic)); return

    if same_topic(topic, TOP_SAMPLE_PERIOD_SET):  handle_sample_period_set(js); return
    if same_topic(topic, TOP_SUMMARY_PERIOD_SET): handle_summary_period_set(js); return

    if same_topic(topic, TOP_CFG_GET):            handle_cfg_get(); return
    if same_topic(topic, TOP_CFG_SET):            handle_cfg_set(js); return

    if same_topic(topic, TOP_CAL_START):          handle_calibration_start(); return

    if same_topic(topic, TOP_Q_ZERO):             handle_q_zero(); return
    if same_topic(topic, TOP_Q_TARGET):           handle_q_target(js); return
    if same_topic(topic, TOP_Q_GET):              handle_q_get(); return
    if same_topic(topic, TOP_Q_SET):              handle_q_set(js); return

    if same_topic(topic, TOP_LED):                handle_led(js); return

    if same_topic(topic, TOP_TEST_DEPTH):         test_depth(); return
    if same_topic(topic, TOP_TEST_IMU):           test_imu(); return
    if same_topic(topic, TOP_TEST_CAP):           test_cap(); return
    if same_topic(topic, TOP_TEST_ALL):           test_all(); return
    if same_topic(topic, TOP_OFFSETS_ALL):        report_all_offsets(); return

    ack(bm, "ERR: unhandled topic {}".format(topic))

def _sub_handler(topic_name, handler_fn, needs_json=False):
    """Return a per-topic callback compatible with bm_serial.bristlemouth_sub().
       It ACKs receipt, decodes payload (optionally JSON), and calls your handler."""
    def _cb(node_id, msg_type, version, topic_len, topic, data_len, data):
        # Confirm we received this command
        ack_rx(bm, topic_name)
        text = safe_text(data)
        js = None
        if needs_json:
            js = parse_json(text) if text else None
            if js is None:
                ack(bm, "ERR: bad JSON for topic {}".format(topic_name))
                return
        try:
            if needs_json:
                handler_fn(js)
            else:
                handler_fn()
        except Exception as e:
            ack(bm, "{} ERR: {}".format(topic_name, e))
    return _cb

# Register subs
# for t in (
#     TOP_SAMPLE_PERIOD_SET, TOP_SUMMARY_PERIOD_SET,
#     TOP_CFG_GET, TOP_CFG_SET,
#     TOP_CAL_START,
#     TOP_Q_ZERO, TOP_Q_TARGET, TOP_Q_GET, TOP_Q_SET,
#     TOP_LED,
#     TOP_TEST_DEPTH, TOP_TEST_IMU, TOP_TEST_CAP, TOP_TEST_ALL, TOP_OFFSETS_ALL,
# ):
#     try: bm.bristlemouth_sub(t, lambda *a, **k: None)
#     except Exception: pass
# try: bm.bristlemouth_tap(tap_router)
# except Exception: pass
# --- Register each topic with a direct per-topic callback (no tap router) ---
try:
    bm.bristlemouth_sub(TOP_SAMPLE_PERIOD_SET,  _sub_handler(TOP_SAMPLE_PERIOD_SET,  handle_sample_period_set,  needs_json=True))
    bm.bristlemouth_sub(TOP_SUMMARY_PERIOD_SET, _sub_handler(TOP_SUMMARY_PERIOD_SET, handle_summary_period_set, needs_json=True))

    bm.bristlemouth_sub(TOP_CFG_GET,            _sub_handler(TOP_CFG_GET,            handle_cfg_get,           needs_json=False))
    bm.bristlemouth_sub(TOP_CFG_SET,            _sub_handler(TOP_CFG_SET,            handle_cfg_set,           needs_json=True))

    bm.bristlemouth_sub(TOP_CAL_START,          _sub_handler(TOP_CAL_START,          handle_calibration_start, needs_json=False))

    bm.bristlemouth_sub(TOP_Q_ZERO,             _sub_handler(TOP_Q_ZERO,             handle_q_zero,            needs_json=False))
    bm.bristlemouth_sub(TOP_Q_TARGET,           _sub_handler(TOP_Q_TARGET,           handle_q_target,          needs_json=True))
    bm.bristlemouth_sub(TOP_Q_GET,              _sub_handler(TOP_Q_GET,              handle_q_get,             needs_json=False))
    bm.bristlemouth_sub(TOP_Q_SET,              _sub_handler(TOP_Q_SET,              handle_q_set,             needs_json=True))

    bm.bristlemouth_sub(TOP_LED,                _sub_handler(TOP_LED,                handle_led,               needs_json=True))

    bm.bristlemouth_sub(TOP_TEST_DEPTH,         _sub_handler(TOP_TEST_DEPTH,         test_depth,               needs_json=False))
    bm.bristlemouth_sub(TOP_TEST_IMU,           _sub_handler(TOP_TEST_IMU,           test_imu,                 needs_json=False))
    bm.bristlemouth_sub(TOP_TEST_CAP,           _sub_handler(TOP_TEST_CAP,           test_cap,                 needs_json=False))
    bm.bristlemouth_sub(TOP_TEST_ALL,           _sub_handler(TOP_TEST_ALL,           test_all,                 needs_json=False))
    bm.bristlemouth_sub(TOP_OFFSETS_ALL,        _sub_handler(TOP_OFFSETS_ALL,        report_all_offsets,       needs_json=False))
except Exception as e:
    print("SUB register error:", e)

# send SUBs so the network knows you want these topics
for t in (TOP_SAMPLE_PERIOD_SET, TOP_SUMMARY_PERIOD_SET, TOP_CFG_GET, TOP_CFG_SET,
          TOP_CAL_START, TOP_Q_ZERO, TOP_Q_TARGET, TOP_Q_GET, TOP_Q_SET,
          TOP_LED, TOP_TEST_DEPTH, TOP_TEST_IMU, TOP_TEST_CAP, TOP_TEST_ALL, TOP_OFFSETS_ALL):
    try:
        bm.bristlemouth_sub(t, None)   # fn unused by this driver
    except Exception as e:
        print("SUB error for", t, "->", e)

# register the TAP so you actually receive messages
try:
    bm.bristlemouth_tap(tap_router)
except Exception as e:
    print("TAP register error:", e)


# -------------- Files, timing, logs --------------
SD_LOG_RAW = "sensor_data_raw.log"
SD_LOG_AGG = "sensor_data_aggregated.log"

_start_ms = time.monotonic() * 1000.0
_next_poll_ms = _start_ms + SENSOR_POLL_INTERVAL_MS

# -------------- Main loop ------------------------
_set_led(LED_COLORS["off"])

FS_WRITABLE = fs_is_writable()
print("[MODE]", "Device-write (USB hidden)" if FS_WRITABLE else "Host-edit (USB visible / RO)")

while True:
    try:
        bm.bristlemouth_process(0.25)
    except Exception:
        pass

    now_ms = time.monotonic() * 1000.0

    # Auto-restore LED after a TX pulse
    if _tx_until_ms and now_ms >= _tx_until_ms:
        _tx_until_ms = 0.0
        _set_led(_tx_restore)

    # >>> skip sensor reads if calibration is running
    if CAL_ACTIVE:
        time.sleep(0.01)
    else:
        if now_ms >= _next_poll_ms:
            _next_poll_ms += SENSOR_POLL_INTERVAL_MS

            # Read Keller
            pressure = None; temperature = None
            if keller_ok and keller.read():
                try:
                    pressure = float(keller.pressure()) + kel_bias_p
                    temperature = float(keller.temperature()) + kel_bias_t
                except Exception:
                    pressure = None; temperature = None

            # IMU orientation
            # debug_dump_once()

            try:
                # If you can access the underlying adafruit_bno055 object:
                st = imu.bno.calibration_status  # (sys, gyro, accel, mag)
                print("[BNO cal] sys={}, g={}, a={}, m={}".format(*st))
            except Exception:
                pass

            roll = pitch = yaw = tilt = None
            if imu_ok:
                try:
                    roll, pitch, yaw, tilt = imu.get_corrected_orientation()
                    roll = float(roll); pitch=float(pitch); yaw=float(yaw); tilt=float(tilt)
                except Exception:
                    roll = pitch = yaw = tilt = None

            # Capacitive
            cap_value = None
            if cap_ok:
                try:
                    raw = float(capacitive.read_value())
                    cap_value = cap_off + cap_scl*raw
                except Exception:
                    cap_value = None

            # Update stats
            def upd(name, val):
                if is_num(val): stats[name].update(val)
            upd("pressure",    pressure)
            upd("temperature", temperature)
            upd("capacitive",  cap_value)
            upd("roll",        roll)
            upd("pitch",       pitch)
            upd("yaw",         yaw)
            upd("tilt",        tilt)

            # ---- HARD-CODED PRECISION WHEN SAVING/PRINTING ----
            def f3(x): return "NA" if x is None else f"{float(x):.3f}"  # pressure (dbar)
            def f2(x): return "NA" if x is None else f"{float(x):.2f}"  # temperature (°C)
            def f1(x): return "NA" if x is None else f"{float(x):.1f}"  # angles (deg)
            def i0(x): return "NA" if x is None else str(int(round(float(x))))  # cap (int)

            parts = [
                str(int(now_ms)),
                f3(pressure),
                f2(temperature),
                i0(cap_value),
                f1(roll),
                f1(pitch),
                f1(yaw),
                f1(tilt),
            ]
            raw_line = ",".join(parts)
            # Log raw CSV (formatted)
            bm.spotter_log(SD_LOG_RAW, raw_line)                  # SD
            bm.spotter_print("IMU/DEPTH/CAP," + raw_line)         # BM console stream (lightweight)

            print("time_ms,pressure,temperature,cap_value,roll,pitch,yaw,tilt")
            print(raw_line); print()
            led_working()

        # Summary
        if now_ms - _start_ms >= AGGREGATION_INTERVAL_MS:
            def pack_stat(s):
                n = s.n; mean = s.mean if n>0 else float("nan")
                sd  = s.stdev if n>1 else float("nan")
                return "{},{:.3f},{:.3f}".format(n, mean, sd)

            agg_line = "IMU_00," + ",".join([
                str(int(now_ms)),
                pack_stat(stats["pressure"]),
                pack_stat(stats["temperature"]),
                pack_stat(stats["capacitive"]),
                pack_stat(stats["roll"]),
                pack_stat(stats["pitch"]),
                pack_stat(stats["yaw"]),
                pack_stat(stats["tilt"]),
            ])
            bm.spotter_log(SD_LOG_AGG, agg_line)                  # SD (summary)
            try: bm.spotter_tx(agg_line.encode("utf-8"))          # optional uplink (heavier)
            except Exception: pass
            print("[TX]", agg_line)

            _start_ms = now_ms
            for s in stats.values(): s.reset()
            led_tx_pulse(300)                         # brief cyan pulse, then restore


    time.sleep(0.01)
