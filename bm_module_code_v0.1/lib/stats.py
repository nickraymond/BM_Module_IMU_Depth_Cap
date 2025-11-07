import math

class SensorStats:
    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.stdev = 0.0

    def update(self, value):
        self.n += 1
        new_mean = self.mean + (value - self.mean) / self.n
        self.stdev = math.sqrt(((self.n - 1) * self.stdev**2 + (value - self.mean) * (value - new_mean)) / self.n)
        self.mean = new_mean

    def reset(self):
        self.n, self.mean, self.stdev = 0, 0.0, 0.0
