import initialise_hardware as initialise_hardware
import time

devices = initialise_hardware.init()
# devices["drive"].drive(0)
# devices["drive"].steering(-90)
# time.sleep(2)
# devices["drive"].steering(90)
# time.sleep(2)
# devices["drive"].steering(0)

devices['gyro'].update_angle()
curr_angle = devices['gyro'].angle_z()
devices['drive'].steering(90)
time.sleep(0.5)

while True:
    devices['gyro'].update_angle()
    curr_angle = devices['gyro'].angle_z()
    devices['drive'].drive(200)