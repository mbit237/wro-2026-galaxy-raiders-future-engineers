import initialise_hardware
import time

devices = initialise_hardware.init()

prev_time = time.time()
while time.time() - prev_time < 8:
    devices['gyro'].update_angle()
    curr_angle = devices['gyro'].angle_z()
    devices['drive'].steer_p(0, curr_angle, 200)
    print(curr_angle)

devices['drive'].drive(0)
devices['drive'].steering(0)