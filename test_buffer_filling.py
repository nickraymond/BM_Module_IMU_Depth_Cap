# filename: test if the buffer is getting full
import time
import board
import neopixel
import busio
from bm_serial import BristlemouthSerial

# Setup NeoPixel LED
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
pixel.brightness = 0.3

# Define LED colors
led_colors = {
    "transmitting": (0, 255, 0),  # Green for successful transmission
    "error": (255, 0, 0),         # Red for errors
}

# Initialize BristlemouthSerial for UART communication
bm = BristlemouthSerial()

# Main loop
while True:
    try:
        # Get the tick time (time since MCU turned on)
        tick_time = time.monotonic()

        # Create a message with three new lines

        # 10x messages
        #message = f"Test\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\n"

        # 5x messages
        message = f"\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\n"

        # 4x messages
        message = f"\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\nTick Time: {tick_time:.2f}100,100,100,100,100,100,100,100\n"


        # Transmit the message over UART
        print(message)
        #bm.spotter_tx(message.encode("utf-8"))
        bm.spotter_log("test.log",message)

        # Flash the NeoPixel green
        pixel.fill(led_colors["transmitting"])
        time.sleep(0.5)  # Flash duration
        pixel.fill((0, 0, 0))  # Turn off the LED

        # Wait 5 seconds before sending the next message
        time.sleep(1)

    except Exception as e:
        # Handle errors, flash red LED
        print(f"Error: {e}")
        pixel.fill(led_colors["error"])
        time.sleep(1)
        pixel.fill((0, 0, 0))
