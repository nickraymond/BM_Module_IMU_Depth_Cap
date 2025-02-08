# filename: example_capacitive_touch.py
# capacitive sensing example on pin A0, requires pull down 1M ohm resistor
import time
import touchio
import board

# Initialize the capacitive touch pin
touch_pin = touchio.TouchIn(board.A0)

# Print capacitive touch value to the screen
print("Starting capacitive touch test. Touch the sensor!")

while True:
    # Read the raw touch value
    touch_value = touch_pin.raw_value

    # Print the value to the console
    print(f"Touch Value: {touch_value}")

    # Delay to make the output more readable
    time.sleep(0.5)
