# filename: code_00_IMU_depth_aggregating.py
# this works, use as a reference for
import time
import board
import busio
import math
from KellerLD import KellerLD
#from adafruit_bno055 import BNO055_I2C
import adafruit_bno055


# Define your control variables
DEFAULT_SENSOR_AGG_PERIOD_MIN = 0.5  # 30 seconds for quick testing
SENSOR_AGG_PERIOD_MS = DEFAULT_SENSOR_AGG_PERIOD_MIN * 60.0 * 1000.0  # Converted to milliseconds
SENSOR_POLL_INTERVAL_MS = 250  # Polling interval set to 4 Hz = 0.25 seconds
MAX_SENSOR_SAMPLES = int(SENSOR_AGG_PERIOD_MS / SENSOR_POLL_INTERVAL_MS)

# Initialize I2C
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialize KellerLD sensor
sensor = KellerLD(i2c)

# Initialize BNO055 IMU sensor
imu = adafruit_bno055.BNO055_I2C(i2c)

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
            # Print data on one line like the example
            print(f"DATA | Pressure: {pressure:.4f} bar | Temperature: {temperature:.2f} °C", end=" | ")

        else:
            print("Failed to read from Keller sensor.")

        # Read from BNO055 IMU sensor (Euler angles: roll, pitch, yaw)
        euler = imu.euler  # Returns a tuple (roll, pitch, yaw)
        if euler:
            bno055_euler_readings.append(euler)
            # Print Euler angles
            print(f"Roll: {euler[0]:.2f} | Pitch: {euler[1]:.2f} | Yaw: {euler[2]:.2f}")

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

            # Print the calculated statistics
            # Print the calculated statistics using str.format()
            print("STATS | Pressure - Mean: {:.4f} bar, StdDev: {:.4f} | Temperature - Mean: {:.2f} °C, StdDev: {:.2f} °C | Roll - Mean: {:.2f}, StdDev: {:.2f} | Pitch - Mean: {:.2f}, StdDev: {:.2f} | Yaw - Mean: {:.2f}, StdDev: {:.2f}".format(
                pressure_mean, pressure_stdev, temp_mean, temp_stdev, roll_mean, roll_stdev, pitch_mean, pitch_stdev, yaw_mean, yaw_stdev))


            # Clear readings for the next aggregation period
            keller_pressure_readings.clear()
            keller_temperature_readings.clear()
            bno055_euler_readings.clear()

        time.sleep(SENSOR_POLL_INTERVAL_MS / 1000)  # Wait for the next polling interval
# Write your code here :-)
