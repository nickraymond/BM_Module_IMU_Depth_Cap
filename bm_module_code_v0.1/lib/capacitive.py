import touchio

class CapacitiveSensor:
    def __init__(self, pin):
        self.touch_pin = touchio.TouchIn(pin)

    def read_value(self):
        return self.touch_pin.raw_value
