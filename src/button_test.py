import pigpio
import time

# Initialize pigpio
pi = pigpio.pi()

BUTTON_PIN = 17

# Set pin 17 as input
pi.set_mode(BUTTON_PIN, pigpio.INPUT)

# Optional: Set pull-up resistor for cleaner input
pi.set_pull_up_down(BUTTON_PIN, pigpio.PUD_UP)

print("Button test starting. Press the button at pin 17...")

prev_state = pi.read(BUTTON_PIN)
while True:
    state = pi.read(BUTTON_PIN)
    
    # Detect state change
    if state != prev_state:
        if state == 0:
            print("Button PRESSED")
        else:
            print("Button RELEASED")
        prev_state = state
    
    time.sleep(0.05)