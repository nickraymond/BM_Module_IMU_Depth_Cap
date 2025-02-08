# **Tilt Monitoring System - RP2040**

This project is a **tilt and environmental monitoring system** using an **RP2040 microcontroller**, a **BNO055 IMU**, and a **KellerLD pressure sensor**. It continuously measures **tilt, pressure, temperature, and acceleration**, logs the data, and transmits it using **BristlemouthSerial (BM)**.

---

## **✩ Features**
- ✅ **Real-time tilt measurement** using BNO055 IMU.
- ✅ **Pressure & temperature monitoring** with KellerLD sensor.
- ✅ **Capacitive touch sensing** for additional input.
- ✅ **Statistical aggregation** for better data analysis.
- ✅ **Data logging & transmission** via **BristlemouthSerial**.
- ✅ **LED status indicators** for system states.
- ✅ **Garbage collection optimization** for memory stability.

---

## **✩ Hardware Components**
| Component            | Purpose |
|----------------------|---------|
| **RP2040 Board**     | Main microcontroller (Raspberry Pi Pico or similar) |
| **BNO055 IMU**       | Measures Euler angles (yaw, pitch, roll) & acceleration |
| **KellerLD Sensor**  | Measures **pressure** and **temperature** |
| **BristlemouthSerial** | Handles logging and transmitting data |
| **NeoPixel LED**     | Provides **visual feedback** (status indicators) |
| **Capacitive Touch Sensor** | Reads input for interactive control |

---

## **✩ Setup & Wiring**
### **1⃣ Wiring Diagram**
| **RP2040 Pin** | **Component** | **Description** |
|---------------|--------------|----------------|
| `A0` | Capacitive Touch | Reads capacitive input |
| `I2C SDA` | BNO055 & KellerLD | Sensor communication |
| `I2C SCL` | BNO055 & KellerLD | Sensor communication |
| `NEOPIXEL` | LED | Status indicator |
| `TX/RX` | BristlemouthSerial | Data transmission |

### **2⃣ Install Required Libraries**
Copy the following **CircuitPython libraries** into the `lib/` folder on your RP2040:
- `adafruit_bno055.mpy`
- `KellerLD.py`
- `bm_serial.py`
- `neopixel.mpy`
- `touchio.mpy`
- `math.mpy`
- `gc.mpy`

---

## **✩ How It Works**
1. **Sensor Initialization**
   - Initializes **BNO055** (IMU) and **KellerLD** (pressure & temperature).
   - **Capacitive sensor** is set up for user interaction.

2. **Data Collection & Processing**
   - Polls data **every 1 second** (configurable).
   - Computes **tilt angle** from roll and pitch.
   - Updates running **statistical values** (mean, standard deviation).

3. **Data Logging & Transmission**
   - Logs data locally in **tilt_data.log**.
   - Transmits aggregated stats every **5 minutes**.

4. **LED Status Indication**
   - **Green (✅ Success):** System running correctly.
   - **Red (❌ Error):** Sensor failure.
   - **Blue (🔄 Working):** Active data polling.
   - **Cyan (🛡️ Transmitting):** Sending data.

---

## **✩ Code Overview**
### **1⃣ `SensorStats` Class**
- Maintains **rolling statistics** for each sensor.

### **2⃣ `calculate_tilt()` Function**
- Computes **absolute tilt** based on **roll & pitch** angles.

### **3⃣ Main Loop**
- Polls **capacitance, pressure, temperature, IMU angles, acceleration**.
- Logs data & transmits aggregated results.
- Runs **garbage collection (`gc.collect()`)** to free memory.

---

## **✩ Configuration Options**
Modify the following variables in `code.py`:

| **Variable** | **Default Value** | **Purpose** |
|-------------|----------------|-------------|
| `SENSOR_POLL_INTERVAL_MS` | `1000` ms | Sensor poll interval (1 second) |
| `DEFAULT_SENSOR_AGG_PERIOD_MS` | `5 * 60 * 1000` | Aggregation period (5 minutes) |
| `pixel.brightness` | `0.3` | LED brightness |

---

## **✩ Example Data Output**
```
DATA | Cap.: 2300 | Pres: 101325 μmbar | Temp: 24.3 °C | Tilt: 5.2° | Roll: 3.1° | Pitch: 2.0° | Yaw: 180.5° | Accel X: 0.02 | Accel Y: 0.01 | Accel Z: -9.81 | Delta: 1000 ms
Publishing stats: 2300, 101325, 24.3, 5.2, 3.1, 2.0, 180.5, 0.02, 0.01, -9.81
Free memory: 65328 bytes
```

---

## **✩ Future Improvements**
💚 **Add SD card logging** for long-term data storage.  
💚 **Implement sleep mode** for power efficiency.  
💚 **Expand sensor compatibility** (e.g., GPS, depth sensors).  

---

## **✩ License**
This project is open-source under the **MIT License**.  

---

### **📌 Notes**
- **Ensure all libraries are correctly installed.**
- **Check wiring carefully** before running the program.
- **Use `REPL` for debugging output** in CircuitPython.

🚀 **Enjoy your RP2040 tilt monitoring system!** 🚀

