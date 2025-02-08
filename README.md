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

## **✩ Data Logging and Transmission**
### **1⃣ Spotter SD Card Logging**
- The data collected is saved to the **Spotter SD card** under the directory:
  ```
  /Volumes/NO NAME/bm/node_id/tilt_data.log
  ```
- Example of a log file format without GPS fix:
  ```
  267290t | 123379t,1044,996948,22.5,5.00,-175.1,-1.0,142.9,0.05,0.13,-0.16
  ```
  - **Timestamp (millis)**
  - **Capacitive sensor value**
  - **Pressure (μmbar)**
  - **Temperature (°C)**
  - **Tilt (°)**
  - **Roll, Pitch, Yaw (°)**
  - **Acceleration X, Y, Z (m/s²)**

- Example of a log file format with GPS fix:
  ```
  2024-12-07T00:35:36.207Z | 124372t,1053,996948,22.5,5.00,-175.1,-1.0,142.9,0.05,0.14,-0.18

  ```
  - **Timestamp (Epoch time)**
  - **Timestamp (millis)**
  - **Capacitive sensor value**
  - **Pressure (μmbar)**
  - **Temperature (°C)**
  - **Tilt (°)**
  - **Roll, Pitch, Yaw (°)**
  - **Acceleration X, Y, Z (m/s²)**

### **2⃣ Aggregated Data Logging to SD**
- The system aggregates data over a 5-minute period and saves it separately in:
  ```
  /data/tilt_monitoring/aggregated_stats.log
  ```
- Example format of aggregated data:
  ```
  1698759600, 2302.5, 5.3, 101322, 24.35, 180.6, -9.81
  ```
  - **Mean values over 5 minutes** for **pressure, temperature, tilt, etc.**

### **3⃣ BM Serial Transmission Format**
- The data is also sent over **BristlemouthSerial (BM)** via cellular.
- Example of **raw BM serial data transmission**:
  ```
  $BM,1698759260,2300,101325,24.3,5.2,3.1,2.0,180.5,0.02,0.01,-9.81*7F
  ```
- When **parsed into ASCII**, it looks like:
  ```
  BM,1698759260,2300,101325,24.3,5.2,3.1,2.0,180.5,0.02,0.01,-9.81
  ```
  - `BM` → **Bristlemouth Protocol Identifier**
  - **Timestamp, sensor values, tilt, acceleration**
  - **Checksum (`*7F`)** for data integrity

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

