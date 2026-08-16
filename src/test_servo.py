import initialise_hardware as initialise_hardware
import time

devices = initialise_hardware.init()
devices["drive"].drive(0)
devices["drive"].steering(-90)
time.sleep(2)
devices["drive"].steering(90)
time.sleep(2)
devices["drive"].steering(0)
