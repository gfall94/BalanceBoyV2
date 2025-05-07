import logging
import time
import numpy as np
import spidev
import ws2812

COLORS = {
    "red": [0, 255, 0],
    "green": [255, 0, 0],
    "blue": [0, 0, 255],
    "white": [255, 255, 255],
    "black": [0, 0, 0],
    "yellow": [255, 255, 0],
    "cyan": [255, 0, 255],
    "magenta": [0, 255, 255]
}

class EYES:
    def __init__(self, name = "Eyes", logging_level = logging.INFO):
        self.frequency = 0.0
        self.now = 0.0
        self.last_time = 0.0

        self.spi = spidev.SpiDev()

        # Jedes NeoHex hat 37 LEDs, beide zusammen 74
        self.num_leds_per_module = 37
        self.num_leds_total = 74

        self.n_rows = 7
        self.n_columns = 13
        self.rows = [
                     range(33,37),
                     range(28,33),
                     range(22,28),
                     range(15,22),
                     range(9,15),
                     range(4,9),
                     range(0,4)]
        self.columns = [
                     [21],
                     [14,27],
                     [8,20,32],
                     [3,13,26,36],
                     [7,19,31],
                     [2,12,25,35],
                     [6,18,30],
                     [1,11,24,34],
                     [5,17,29],
                     [0,10,23,33],
                     [4,16,28],
                     [9,22],
                     [15]]

        self.leds = [COLORS["black"]] * self.num_leds_total
        self.leds_last = [COLORS["white"]] * self.num_leds_total

        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging_level)

        try:
            self.spi.open(0, 0)
            ws2812.write2812(self.spi, self.leds)

            self.logger.info("Erfolgreich initialisiert.")
        except Exception as e:
            self.logger.critical(str(e))
            self.bno = None

    def loop(self):
            self.now = time.perf_counter()
            self.frequency = (1 / (self.now - self.last_time))
            self.last_time = self.now

            try:
                ws2812.write2812(self.spi, [[10, 0, 10]])  # [G, R, B] :contentReference[oaicite:11]{index=11}

                self.logger.debug("I'm alive")
            except Exception as e:
                self.logger.error(str(e))

    def clear(self):
        ws2812.write2812(self.spi, [COLORS["black"]] * self.num_leds_total)

    def clear_range(self, start, end):
        for i in range(start, end):
            if 0 <= i < self.num_leds_total:
                self.leds[i] = COLORS["black"]

    def single(self,module, idx, color_name):
        # Startindex fürs Modul
        offset = module * self.num_leds_per_module
        self.leds[idx+offset] = COLORS[color_name]

    def fill_range(self, start, end, color_name):
        for i in range(start, end):
            if 0 <= i < self.num_leds_total:
                self.leds[i] = COLORS[color_name]

    def line(self, module_name="none", orientation="vertical", color1_name="white", value1=0.0, min_value1=0.0, max_value1=0.0, color2_name="white", value2=0.0, min_value2=0.0, max_value2=0.0, comb_color_name="white"):
        color1 = COLORS.get(color1_name, COLORS["white"])
        color2 = COLORS.get(color2_name, COLORS["white"])
        comb_color = COLORS.get(comb_color_name, COLORS["white"])
        value1 = np.clip(value1, min_value1, max_value1)
        value2 = np.clip(value2, min_value2, max_value2)

        module=0
        if module_name=="left":
            module = 1
        elif module_name=="right":
            module = 0

        # Startindex fürs Modul
        offset = module * self.num_leds_per_module

        if orientation == "horizontal":
            pos = int(np.interp(value1, [min_value1, max_value1], [0, self.n_rows - 1]))
            horizontal = self.rows[pos]
            for i in horizontal:
                self.leds[offset + i] = color1

        elif orientation == "vertical":
            pos = int(np.interp(value1, [min_value1, max_value1], [0, self.n_columns - 1]))
            vertical = self.columns[pos]
            for i in vertical:
                self.leds[offset + i] = color1

        elif orientation == "cross":
            pos = int(np.interp(value1, [min_value1, max_value1], [0, self.n_columns - 1]))
            vertical = self.columns[pos]
            for i in vertical:
                self.leds[offset + i] = color1

            pos = int(np.interp(value2, [min_value2, max_value2], [0, self.n_rows - 1]))
            horizontal = self.rows[pos]
            for i in horizontal:
                self.leds[offset + i] = color2
                if i in vertical:
                    self.leds[offset + i] = comb_color

    def show(self):
        if not self.leds == self.leds_last:
            ws2812.write2812(self.spi, self.leds)
            for i in range(self.num_leds_total):
                self.leds_last[i] = self.leds[i]