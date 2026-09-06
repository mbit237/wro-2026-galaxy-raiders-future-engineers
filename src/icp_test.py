import time

import initialise_hardware as initialise_hardware
import initialise_pose as initialise_pose
import telemetry_client as telemetry_client
import sensors as sensors
import odometry as odometry
import point_cloud_localisation as localisation
import complementary_filter as complementary_filter
import navigation as navigation
import led as led
import rpicam as rpicam
from paths import open_first_path, cw_paths, ccw_paths

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 12 
SEGMENT_DIVIDER = 1700
STOPPING_TOP_POS = 1800
STOPPING_BOTTOM_POS = 1300

MODE = "obstacle"

devices = initialise_hardware.init()
nav = navigation.Navigation(devices)
cam = rpicam.Rpicam()

if USE_TELEMETRY:
    telemetry_client.connect()

devices['led'].all_off()
time.sleep(1)
devices['led'].red_on()
print("wait for button")
# # display LED colour to show it is ready and the mode (obstacle or open)
while True:
    if devices["pi"].read(17) == 0:
        time.sleep(0.5)
        break 

devices['led'].red_off()

devices['lidar'].flush()
pose = initialise_pose.open(devices)
print('initial pose:', pose)

# deliberate error
pose[0] += 40
pose[1] += 30
pose[2] += 4

print('initial pose w err:', pose)
devices['led'].green_on()

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

end_time = time.time() + 5
loc_count = 0
loop_count = 0
while time.time() < end_time:
    loop_count += 1
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        loc_count += 1
        # pose = complementary_filter.merge(odometry_pose, localised_pose)
        pose = odometry_pose
        print(f"odo_pose: {odometry_pose}")
        print(f" localised_pose: {localised_pose}")
    else:
        pose = odometry_pose

    color = cam.detect_blob() # always have updated image

print('waiting', devices['lidar'].uart.in_waiting)
print('loc_count', loc_count)
print('loop_count', loop_count, loop_count / 5)
nav.stop()

    





