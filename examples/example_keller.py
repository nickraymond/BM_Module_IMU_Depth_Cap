# Keller 1000m pressure sensor
import time
import board
import busio
from KellerLD import KellerLD

# Initialize I2C
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialize KellerLD sensor
sensor = KellerLD(i2c)

if not sensor.init():
    print("Failed to initialize Keller LD sensor!")
    while True:
        pass

print(sensor)

# Main loop
while True:
    if sensor.read():
        print(f"Pressure: {sensor.pressure():.4f} bar, Temperature: {sensor.temperature():.2f} °C")
    else:
        print("Failed to read from sensor.")

    time.sleep(1)
# Write your code here :-)
