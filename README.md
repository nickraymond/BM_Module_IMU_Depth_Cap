# **Tilt + Depth Monitoring System — RP2040 + Bristlemouth**

A compact **tilt and environmental monitoring node** built around the **Adafruit QT Py RP2040**, a **BNO055 IMU**, and a **KellerLD pressure sensor**.  
It logs **tilt, pressure, temperature, and capacitive data**, writes to the **Spotter SD card**, and transmits periodic summaries through the **Bristlemouth Bus (BM)** network.

---

## ✹ Core Features
- **Real-time orientation** from BNO055 (roll, pitch, yaw, tilt)
- **Depth and temperature** via KellerLD pressure sensor
- **Capacitive input** sensing for water/contact detection
- **Configurable sample and summary periods**
- **On-demand calibration & orientation reset** over BM Bus
- **Local SD logging + BM transmission** of aggregated data
- **LED status indication** for active, error, and transmit states
- **Robust JSON-based configuration and persistent offsets**

---

## ✹ Hardware Overview

| Component | Function |
|------------|-----------|
| **Adafruit QT Py RP2040** | Main MCU |
| **BNO055 IMU (I²C)** | Roll, pitch, yaw, tilt |
| **KellerLD Pressure Sensor (I²C)** | Pressure + temperature |
| **Capacitive Sensor (A0)** | Water/Touch input |
| **NeoPixel (on-board)** | Status indicator |
| **BristlemouthSerial (UART)** | Command + telemetry bus |

### Wiring Summary
| RP2040 Pin | Connection | Description |
|-------------|-------------|-------------|
| `A0` | Capacitive sensor | Analog input |
| `SDA` / `SCL` | IMU + Keller | Shared I²C bus |
| `TX` / `RX` | Bristlemouth Bus | Serial data link |
| `NEOPIXEL` | On-board LED | System feedback |

---

## ✹ Data Flow Summary

| Task | Frequency | Output |
|------|------------|--------|
| Sensor sampling | Configurable (`sample_period_s`) | Raw CSV (Spotter SD) + BM `spotter/printf` |
| Aggregation summary | Configurable (`summary_period_s`) | Stats (Spotter SD + BM uplink) |
| Calibration & offsets | On command | Stored in `/config/system.json` |

All configuration and calibration data persist between power cycles.

---

## ✹ LED Status Reference

| Color | Meaning |
|--------|----------|
| 🔵 Blue | Sampling / working |
| 🟢 Green | Normal / success |
| 🔴 Red | Sensor or config error |
| ⚪ White | LED on command |
| 🟣 Cyan | Transmitting summary (auto-restore) |

---

## ✹ BM Bus Commands

You can control and query the device using these BM topics:

| Command Topic | Payload Example | Description |
|----------------|----------------|--------------|
| `device/led` | `{"led":"on","color":"white"}` | Turn LED on/off/blink in chosen color |
| `device/config/get` | `{}` | Return full config (JSON) |
| `device/config/set` | `{"sample_period_s":1.0,"summary_period_s":300.0}` | Update runtime + persistent config |
| `device/sample_period/set` | `{"sample_period_s":2.0}` | Change raw sampling interval |
| `device/summary_period/set` | `{"summary_period_s":120.0}` | Change aggregation interval |
| `device/sample_period/get` | `{}` | Report current sample period |
| `device/summary_period/get` | `{}` | Report current summary period |
| `device/rates/get` | `{}` | Combined sample + summary period report |
| `sensor/depth/test` | `{}` | Read Keller pressure + temperature once |
| `sensor/imu/test` | `{}` | Read IMU roll, pitch, yaw, tilt once |
| `sensor/cap/test` | `{}` | Read capacitive sensor once |
| `sensor/all/test` | `{}` | Read all sensors once |
| `sensor/all/offsets` | `{}` | Report stored calibration + offsets |
| `imu/calibration/start` | `{}` | Begin IMU calibration routine |
| `imu/quaternion/target` | `{"target_deg":[45,35,0]}` | Apply quaternion offset to target pose |

All commands generate ACK messages via `spotter/printf` visible on the BM Bus.

---

## ✹ Data Format

### Raw Data logged to SD (every `sample_period_s`)

| time_ms | pressure | temperature | cap_value | roll  | pitch | yaw   | tilt  |
| ------- | -------- | ----------- | --------- | ----- | ----- | ----- | ----- |
| ms      | dbar     | °C          | counts    | deg   | deg   | deg   | deg   |
| byte    | float    | float       | byte      | float | float | float | float |

``` 
time_ms,pressure,temperature,cap_value,roll,pitch,yaw,tilt
82451,1.012,24.95,945,-42.3,0.0,4.5,42.3
```

### Aggregated summary logged to SD (every `summary_period_s`)
| tag                | time_ms | pressure_n | pressure_mean | pressure_stdev | temperature_n | temperature_mean | temperature_stdev | capacitive_n | capacitive_mean | capacitive_stdev | roll_n  | roll_mean | roll_stdev | pitch_n | pitch_mean | pitch_stdev | yaw_n   | yaw_mean | yaw_stdev | tilt_n  | tilt_mean | tilt_stdev |
| ------------------ | ------- | ---------- | ------------- | -------------- | ------------- | ---------------- | ----------------- | ------------ | --------------- | ---------------- | ------- | --------- | ---------- | ------- | ---------- | ----------- | ------- | -------- | --------- | ------- | --------- | ---------- |
| literal `"IMU_00"` | ms      | samples    | dbar          | dbar           | samples       | °C               | °C                | samples      | counts          | counts           | samples | deg       | deg        | samples | deg        | deg         | samples | deg      | deg       | samples | deg       | deg        |
| (string)           | byte    | byte       | float         | float          | byte          | float            | float             | byte         | float           | float            | byte    | float     | float      | byte    | float      | float       | byte    | float    | float     | byte    | float     | float      |

```
IMU_00,82451,60,1.013,0.005,60,24.95,0.12,60,43.1,1.4,60,-42.4,0.9,60,0.1,0.1,60,4.4,1.0
```



### Transmission (every `summary_period_s`)
- Summary: logged + transmitted via `spotter_tx()` (BM cellular uplink)
- Same as aggregated summary format above.

## ✹ Configuration File

All persistent settings and offsets are stored on the RP2040 as:

``` 
/config/system.json


Example contents:
```json
{
  "sample_period_s": 1.0,
  "summary_period_s": 300.0,
  "imu_offsets": {
    "magnetometer": [-682, -420, -354],
    "gyroscope": [0, -3, 0],
    "accelerometer": [-25, 83, -35]
  },
  "quaternion_target_deg": [0, 0, 0]
}

```

## ✹ Maintenance & File Access

The system uses a dual-mode boot.py:

- Normal run (no USB) → filesystem writable, device logs + saves configs

- Maintenance mode (A3 → GND at boot) → USB visible for host editing (read-only for code)

Use REPL (Ctrl + C → >>>) or BM commands to verify filesystem access.

Ensure /config exists on fresh devices before running.

## Cheat Sheet of BM Commands
## ✹ BM Bus One-Liners (Copy & Paste)

> Format: `bm pub <topic> <json> text 0`  
> Each command prints an ACK on the BM bus via `spotter/printf`.

### LED / Status
- **LED ON (white)**  
  `bm pub device/led {"led":"on","color":"white"} text 0`


- **LED OFF**  
  `bm pub device/led {"led":"off"} text 0`


- **LED BLINK (green, 500 ms, 5x)**  
  `bm pub device/led {"led":"blink","color":"success","period_ms":500,"count":5} text 0`

---

### Config — Get / Set
- **Get full config**  
  `bm pub device/config/get {} text 0`


- **Set both periods (examples: 1s sample, 300s summary)**  
  `bm pub device/config/set {"sample_period_s":1.0,"summary_period_s":300.0} text 0`  



- **Get sample period**  
  `bm pub device/sample_period/get {} text 0`  



- **Set sample period (2.5 s)**  
  `bm pub device/sample_period/set {"sample_period_s":2.5} text 0`  



- **Get summary period**  
  `bm pub device/summary_period/get {} text 0`  



- **Set summary period (120 s)**  
  `bm pub device/summary_period/set {"summary_period_s":120} text 0`  



- **Get both rates**  
  `bm pub device/rates/get {} text 0`  



---

### Sensors — Quick Tests
- **Depth/Temp test (Keller)**  
  `bm pub sensor/depth/test {} text 0`  



- **IMU test (RPYT)**  
  `bm pub sensor/imu/test {} text 0`  



- **Capacitive test**  
  `bm pub sensor/cap/test {} text 0`  



- **All sensors**  
  `bm pub sensor/all/test {} text 0`  



- **Report stored offsets**  
  `bm pub sensor/all/offsets {} text 0`  



---

### IMU — Calibration & Orientation
- **Start IMU calibration (guided)**  
  `bm pub imu/calibration/start {} text 0`  
  

- _ACKs:_ progress like `CAL mag=.../3 accel=.../3 gyro=.../3`, then `CAL SAVED: mag=(...), gyro=(...), accel=(...)`  
  _(If filesystem RO/missing `/config`, expect `CAL ERR: ...`.)_



- **Zero orientation to current pose**  
  `bm pub imu/quaternion/zero {} text 0`  



- **Set orientation target (R,P,Y in degrees)**  
  `bm pub imu/quaternion/target {"target_deg":[45,35,0]} text 0`  



---

### Filesystem (Optional health check)
- **Writable check**  
  `bm pub device/fs/test {} text 0`  
  _ACK:_ `FS TEST: OK (writable)` _or_ `FS TEST: FAIL -> ...`

**Notes**
> - Period keys: `sample_period_s`, `summary_period_s` (seconds).  
> - All ACKs are emitted via `spotter/printf`; summaries also use `spotter_tx` for uplink.  
> - If you see `... ERR: save failed`, ensure `/config` exists and the device is in **device-write mode** (USB hidden or `boot.py` jumper set).
