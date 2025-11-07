import time
import touchio
import board
import neopixel
import math
import gc
from KellerLD import KellerLD
from adafruit_bno055 import BNO055_I2C
from bm_serial import BristlemouthSerial

# LED color dictionary for better conditional handling
led_colors = {
    "success": (0, 255, 0),
    "error": (255, 0, 0),
    "working": (0, 0, 255),
    "transmitting": (0, 255, 255),
}

# Helper class for running stats
class SensorStats:
    def __init__(self):
        self.n = 0
        self.mean = 0
        self.stdev = 0

    def update(self, new_value):
        if self.n == 0:
            self.n, self.mean, self.stdev = 1, new_value, 0
        else:
            self.n += 1
            new_mean = self.mean + (new_value - self.mean) / self.n
            self.stdev = math.sqrt(((self.n - 1) * self.stdev**2 + (new_value - self.mean) * (new_value - new_mean)) / self.n)
            self.mean = new_mean

    def reset(self):
        self.n = 0
        self.mean = 0
        self.stdev = 0

# Function to calculate tilt angle
def calculate_tilt(roll, pitch):
    """
    Calculate the absolute tilt angle relative to vertical using roll and pitch.

    :param roll: Roll angle in degrees
    :param pitch: Pitch angle in degrees
    :return: Absolute tilt angle in degrees
    """
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    tilt_rad = math.acos(math.sqrt(math.cos(roll_rad)**2 * math.cos(pitch_rad)**2))
    return math.degrees(tilt_rad)

# Initialize the capacitive touch pin
touch_pin = touchio.TouchIn(board.A0)

# Setup the NeoPixel LED
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.brightness = 0.3

# Debug flags
debug_terminal = 1
debug_led = 1

# Initialize BM Serial
bm = BristlemouthSerial()

# Define control variables
DEFAULT_SENSOR_AGG_PERIOD_MS = 5 * 60 * 1000  # 5 minutes in milliseconds
SENSOR_POLL_INTERVAL_MS = 1  # 250 ms (4 Hz)

# Initialize I2C for sensors
i2c = board.I2C()
bno055 = BNO055_I2C(i2c)
sensor = KellerLD(i2c)

# Initialize sensors and set error state if initialization fails
if not sensor.init():
    print("Failed to initialize Keller LD sensor!")
    pixel.fill(led_colors["error"])
    while True:
        pass

# Initialize stats
stats = {
    "capacitive": SensorStats(),
    "pressure": SensorStats(),
    "temperature": SensorStats(),
    "roll": SensorStats(),
    "pitch": SensorStats(),
    "yaw": SensorStats(),
    "accel_x": SensorStats(),
    "accel_y": SensorStats(),
    "accel_z": SensorStats(),
    "tilt": SensorStats(),  # Add tilt to stats
}

# Main loop variables
start_time = time.monotonic() * 1000
next_poll_time = start_time + SENSOR_POLL_INTERVAL_MS
last_debug_time = start_time

while True:
    current_time = time.monotonic() * 1000

    # Sensor polling
    if current_time >= next_poll_time:
        next_poll_time += SENSOR_POLL_INTERVAL_MS

        # Read capacitive value
        capacitive_value = touch_pin.raw_value
        stats["capacitive"].update(capacitive_value)

        # Read from Keller sensor
        if sensor.read():
            pressure = sensor.pressure() * 1_000_000  # Convert pressure to microbar
            temperature = sensor.temperature() * 10  # Scale temperature to 1 decimal place
            stats["pressure"].update(pressure)
            stats["temperature"].update(temperature)

            # Read Euler angles from BNO055
            euler = bno055.euler
            if euler:
                yaw, pitch, roll = [int(x * 10) for x in euler]  # Store angles as scaled integers
                stats["roll"].update(roll)
                stats["pitch"].update(pitch)
                stats["yaw"].update(yaw)

                # Calculate and update tilt
                tilt = calculate_tilt(roll / 10, pitch / 10)  # Convert back to degrees
                stats["tilt"].update(tilt)

            # Read accelerometer data (gravity removed)
            accel = bno055.linear_acceleration  # Returns (x, y, z) with gravity removed
            if accel:
                accel_x, accel_y, accel_z = accel
                stats["accel_x"].update(accel_x)
                stats["accel_y"].update(accel_y)
                stats["accel_z"].update(accel_z)

            # Prepare log message for this poll
            poll_log_message = f"{int(current_time)}t,{capacitive_value},{pressure:.0f},{temperature / 10:.1f},{roll / 10:.1f},{pitch / 10:.1f},{yaw / 10:.1f},{accel_x:.2f},{accel_y:.2f},{accel_z:.2f},{tilt:.2f}"

            # Save sensor poll data to SD card
            bm.spotter_log("sensor_stats.log", poll_log_message)

            # Debugging
            delta_time = current_time - last_debug_time
            print(f"DATA | {poll_log_message} | Time since last debug: {delta_time:.1f} ms")
            #print(f"DATA | Roll: {roll / 10:.1f} | Pitch: {pitch / 10:.1f} | Yaw: {yaw / 10:.1f} | Tilt: {tilt:.1f} | Accel X: {accel_x:.1f} | Accel Y: {accel_y:.1f} | Accel Z: {accel_z:.1f} | Delta: {delta_time:.0f} ms")

            last_debug_time = current_time
            pixel.fill(led_colors["working"])
        else:
            print("Failed to read from Keller sensor.")
            pixel.fill(led_colors["error"])

    # After aggregation period, send statistics
    if current_time - start_time >= DEFAULT_SENSOR_AGG_PERIOD_MS:
        stats_log_message = f"{int(current_time)}t,{stats['capacitive'].mean:.1f},{stats['capacitive'].stdev:.1f},{stats['pressure'].mean:.0f},{stats['pressure'].stdev:.0f},{stats['temperature'].mean / 10:.1f},{stats['temperature'].stdev / 10:.1f},{stats['roll'].mean / 10:.1f},{stats['roll'].stdev / 10:.1f},{stats['pitch'].mean / 10:.1f},{stats['pitch'].stdev / 10:.1f},{stats['yaw'].mean / 10:.1f},{stats['yaw'].stdev / 10:.1f},{stats['accel_x'].mean:.2f},{stats['accel_x'].stdev:.2f},{stats['accel_y'].mean:.2f},{stats['accel_y'].stdev:.2f},{stats['accel_z'].mean:.2f},{stats['accel_z'].stdev:.2f},{stats['tilt'].mean:.2f},{stats['tilt'].stdev:.2f}"

        # Transmit aggregated statistics
        bm.spotter_tx(stats_log_message.encode("utf-8"))
        print(f"Publishing stats: {stats_log_message}")
        pixel.fill(led_colors["transmitting"])

        # Reset stats
        for stat in stats.values():
            stat.reset()

        # Reset start time for the next aggregation period
        start_time = current_time

        # Force garbage collection
        gc.collect()
        print(f"Free memory: {gc.mem_free()} bytes")

    time.sleep(0.01)  # Prevent busy looping
