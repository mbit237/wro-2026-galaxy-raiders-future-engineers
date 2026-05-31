import time

import initialise_hardware
import initialise_pose
import telemetry_client
import sensors
import odometry
import spike_localisation as localisation
import complementary_filter
import navigation
from paths import OPEN_FIRST_PATH

USE_TELEMETRY = True
SPEED = 250 

devices = initialise_hardware.init()
pose = initialise_pose.open(devices)

if USE_TELEMETRY:
    telemetry_client.connect()

print("wait for button")
while True:
    if devices["pi"].read(17) == 0:
        time.sleep(0.5)
        break 

# First path
odometry.reset_pose()
while True:
    sensor_readings = sensors.read()
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
    else:
        pose = odometry_pose

    if navigation.drive_path(OPEN_FIRST_PATH, pose, SPEED):
        if localised_pose:
            break

pose = initialise_pose.confirm_pose(pose, sensor_readings)




