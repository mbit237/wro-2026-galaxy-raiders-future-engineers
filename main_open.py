import time

import initialise_hardware
import initialise_pose
import telemetry_client
import sensors
import odometry
import spike_localisation as localisation
import complementary_filter
import navigation
import led
from paths import open_first_path, cw_paths, ccw_paths

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 8 
SEGMENT_DIVIDER = 1700
STOPPING_TOP_POS = 1800
STOPPING_BOTTOM_POS = 1300

devices = initialise_hardware.init()
nav = navigation.Navigation(devices)
led = led.LED()

if USE_TELEMETRY:
    telemetry_client.connect()


led.red_off()
time.sleep(1)
led.red_on()
print("wait for button")
# # display LED colour to show it is ready and the mode (obstacle or open)
while True:
    if devices["pi"].read(17) == 0:
        time.sleep(0.5)
        break 

pose = initialise_pose.open(devices)
print(pose)

led.red_off()
led.green_on()

# Check if first wall is extended 
if pose[3]:
    open_first_path[0][0] = 300
    open_first_path[1][0] = 300

# Save startin segment
segment = 'top'
if pose[1] < SEGMENT_DIVIDER:
    segment = 'bottom'

# --------------- First path --------------- # 
odometry.reset_pose()
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
    else:
        pose = odometry_pose

    print(pose)
    if nav.drive_path(open_first_path, pose, SPEED):
        if localised_pose:
            break

# ----- Change it such that the while loop only breaks after the pose is confirmed ----- #
pose = initialise_pose.confirm_pose(pose, sensor_readings) 

paths = []
if pose[0] < 1500:
    paths = cw_paths
    print('cw')
else:
    paths = ccw_paths
    print('ccw')
path_idx = 1 # skip first path

print('After first path: ', pose)


# --------------- Main Loop --------------- # 
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print('Localised pose: ', pose)
        
    else:
        pose = odometry_pose
        
    
    path_changed, path_idx = nav.drive_paths(path_idx, paths, pose, SPEED)
    if path_changed:
        print('path_changed: ', path_idx)
    if path_idx >= PATHS_LIMIT:
        if segment == 'bottom' and pose[1] >= STOPPING_BOTTOM_POS:
            break
        elif segment == 'top' and pose[1] >= STOPPING_TOP_POS:
            break

nav.stop()

    





