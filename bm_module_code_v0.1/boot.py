# boot.py — maintenance pin toggle for QT Py RP2040
import board, digitalio, storage, time

# Pick a safe pad (not SDA/SCL, not TX/RX, not NEOPIXEL)
MAINT_PIN = board.A3  # jumper A3 to GND while resetting for host-edit mode

key = digitalio.DigitalInOut(MAINT_PIN)
key.switch_to_input(pull=digitalio.Pull.UP)

# Simple debounce / settle
time.sleep(0.02)
pressed = not key.value  # LOW when jumpered to GND

if pressed:
    # Host-edit mode: CIRCUITPY visible to computer; keep FS read-only to running code
    storage.enable_usb_drive()
    # Do NOT remount RW for code while USB is visible.
else:
    # Device-write mode: hide USB; allow code to write files safely
    storage.disable_usb_drive()
    storage.remount("/", readonly=False)
