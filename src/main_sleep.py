import time
import pigpio

pi = pigpio.pi()
pi.set_mode(13, pigpio.OUTPUT) # red
pi.set_mode(19, pigpio.OUTPUT) # green
pi.set_mode(26, pigpio.OUTPUT) # yellow
pi.write(13, 0)
pi.write(19, 0)
pi.write(26, 0)

while True:
    time.sleep(1)
