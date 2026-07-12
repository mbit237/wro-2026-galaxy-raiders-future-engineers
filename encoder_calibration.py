import initialise_hardware
import time

devices = initialise_hardware.init()

print(devices['encoder'].steps)

devices['drive'].drive(200)
time.sleep(8)
devices['drive'].drive(0)

print(devices['encoder'].steps)

# Readings 
# Actual    Encoder
# 1285      2705    5 seconds
# 2005      4370    8 seconds -- Discarded
# 2075      4365    8 seconds 
# 2070      4357    8 seconds