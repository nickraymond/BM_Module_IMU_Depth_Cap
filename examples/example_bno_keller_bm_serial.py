import time
import board
import busio
import math
import digitalio
from KellerLD import KellerLD
from adafruit_bno055 import BNO055_I2C
from bm_serial import BristlemouthSerial

# Initialize BM Serial
bm = BristlemouthSerial()

# Initialize the onboard LED
led = digitalio.DigitalInOut(board.LED)
led.direction = digitalio.Direction.OUTPUT

# Define your control variables
DEFAULT_SENSOR_AGG_PERIOD_MIN = 0.5  # 30 seconds for quick testing
SENSOR_AGG_PERIOD_MS = DEFAULT_SENSOR_AGG_PERIOD_MIN * 60.0 * 1000.0  # Converted to milliseconds
SENSOR_POLL_INTERVAL_MS = 250  # Polling interval set to 2 seconds
MAX_SENSOR_SAMPLES = int(SENSOR_AGG_PERIOD_MS / SENSOR_POLL_INTERVAL_MS)

# Initialize I2C for both sensors (SDA=GP3, SCL=GP2) if using Raspberry Pi RP2040
#i2c = busio.I2C(scl=board.GP3, sda=board.GP2)

# Initialize I2C for both sensors (SDA, SCL) if using QTPY RP2040
i2c = board.I2C()  # uses board.SCL and board.SDA
bno055 = adafruit_bno055.BNO055_I2C(i2c)

# Initialize KellerLD sensor
sensor = KellerLD(i2c)

# Initialize BNO055 IMU sensor
imu = BNO055_I2C(i2c)  # Correct BNO055 I2C usage

# Initialize the Keller sensor
if not sensor.init():
    print("Failed to initialize Keller LD sensor!")
    while True:
        pass

# Lists to store the sensor readings for the aggregation period
keller_pressure_readings = []
keller_temperature_readings = []
bno055_euler_readings = []

start_time = time.monotonic() * 1000  # Current time in milliseconds

# Helper functions to calculate mean and standard deviation
def calculate_mean(data):
    return sum(data) / len(data) if len(data) > 0 else 0

def calculate_stdev(data, mean):
    variance = sum((x - mean) ** 2 for x in data) / len(data) if len(data) > 0 else 0
    return math.sqrt(variance)

# Main loop to read sensor data
while True:
    # Periodically read the sensors every SENSOR_POLL_INTERVAL_MS milliseconds
    current_time = time.monotonic() * 1000  # Current time in milliseconds

    # Check if it's time to take a sensor reading
    if (current_time - start_time) >= SENSOR_POLL_INTERVAL_MS:
        # Reset the start time for the next poll
        start_time = current_time

        # Read from Keller sensor
        if sensor.read():
            pressure = sensor.pressure()
            temperature = sensor.temperature()
            keller_pressure_readings.append(pressure)
            keller_temperature_readings.append(temperature)

            # Read from BNO055 IMU sensor (Euler angles: roll, pitch, yaw)
            euler = imu.euler  # Returns a tuple (roll, pitch, yaw)

            if euler:
                bno055_euler_readings.append(euler)

                # Log the raw values to sensor_data.log
                raw_log_message = f"{int(current_time)}t,{pressure:.2f},{temperature:.2f},{euler[0]:.2f},{euler[1]:.2f},{euler[2]:.2f}"

                # Turn on the LED before sending the message
                led.value = True

                bm.spotter_log("sensor_data.log", raw_log_message)

                # Turn off the LED after the message is sent
                led.value = False

                # Print data to terminal
                print("DATA | Pressure:", pressure, "bar | Temperature:", temperature, "°C | Roll:", euler[0], "| Pitch:", euler[1], "| Yaw:", euler[2])

            else:
                print("Failed to read from BNO055 IMU sensor.")

        else:
            print("Failed to read from Keller sensor.")

        # If we have reached the maximum number of samples, calculate stats
        if len(keller_pressure_readings) >= MAX_SENSOR_SAMPLES:
            # Calculate the mean and standard deviation for Keller sensor
            pressure_mean = calculate_mean(keller_pressure_readings)
            pressure_stdev = calculate_stdev(keller_pressure_readings, pressure_mean)
            temp_mean = calculate_mean(keller_temperature_readings)
            temp_stdev = calculate_stdev(keller_temperature_readings, temp_mean)

            # For BNO055, calculate mean and stdev for Euler angles (each axis separately)
            roll_angles = [x[0] for x in bno055_euler_readings]  # Roll
            pitch_angles = [x[1] for x in bno055_euler_readings]  # Pitch
            yaw_angles = [x[2] for x in bno055_euler_readings]    # Yaw

            roll_mean = calculate_mean(roll_angles)
            roll_stdev = calculate_stdev(roll_angles, roll_mean)
            pitch_mean = calculate_mean(pitch_angles)
            pitch_stdev = calculate_stdev(pitch_angles, pitch_mean)
            yaw_mean = calculate_mean(yaw_angles)
            yaw_stdev = calculate_stdev(yaw_angles, yaw_mean)

            # Prepare the statistics log message
            stats_log_message = f"{int(current_time)}t,{pressure_mean:.2f},{pressure_stdev:.2f},{temp_mean:.2f},{temp_stdev:.2f},{roll_mean:.2f},{roll_stdev:.2f},{pitch_mean:.2f},{pitch_stdev:.2f},{yaw_mean:.2f},{yaw_stdev:.2f}"

            # Turn on the LED before sending the stats message
            led.value = True

            # Log the statistics to sensor_stats.log
            bm.spotter_log("sensor_stats.log", stats_log_message)

            # Turn off the LED after the message is sent
            led.value = False

            # Print the calculated statistics
            print(f"Publishing stats: {stats_log_message}")

            # Clear readings for the next aggregation period
            keller_pressure_readings.clear()
            keller_temperature_readings.clear()
            bno055_euler_readings.clear()

        time.sleep(SENSOR_POLL_INTERVAL_MS / 1000)  # Wait for the next polling interval
# Write your code here :-)
