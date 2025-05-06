import evdev
from select import select

dev = evdev.InputDevice('/dev/input/event2')  # Dein PS4 Controller Event-Device
dev.grab()  # Optional: exklusiver Zugriff

while True:
    r, w, x = select([dev], [], [], 0.01)  # 10ms Timeout
    if dev in r:
        for event in dev.read():
            if event.type == evdev.ecodes.EV_ABS:
                print(evdev.categorize(event))