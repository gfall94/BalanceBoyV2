import time


class LowPassFilter:
    def __init__(self, cutoff_hz):
        self.cutoff_hz = cutoff_hz
        self.last_output = None
        self.last_time = None

    def compute_alpha(self, cutoff, dt):
        rc = 1 / (2 * 3.1416 * cutoff)
        return dt / (rc + dt)

    def filter(self, input_value):
        now = time.time()

        if self.last_time is None:
            dt = 0.02  # Defaultwert (für z. B. 50 Hz) beim ersten Aufruf
        else:
            dt = now - self.last_time

        self.last_time = now
        alpha = self.compute_alpha(self.cutoff_hz, dt)

        if self.last_output is None:
            self.last_output = input_value
        else:
            self.last_output = alpha * input_value + (1 - alpha) * self.last_output

        return self.last_output
