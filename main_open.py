import time

import initialise_hardware
import initialise_pose
import telemetry_client
import sensors
import odometry
import spike_localisation as localisation
import complementary_filter
import navigation
from paths import open_first_path, cw_paths, ccw_paths

USE_TELEMETRY = True
SPEED = 250 
PATHS_LIMIT = 16

devices = initialise_hardware.init()
nav = navigation.Navigation(devices)
        
if USE_TELEMETRY:
    telemetry_client.connect()

print("wait for button")
# display LED colour to show it is ready and the mode (obstacle or open)
while True:
    if devices["pi"].read(17) == 0:
        time.sleep(0.5)
        break 

pose = initialise_pose.open(devices)

# Check if first wall is extended 
if pose[3]:
    open_first_path[0][0] = 300
    open_first_path[1][0] = 300

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

    if nav.drive_path(open_first_path, pose, SPEED):
        if localised_pose:
            break

# Change it such that the while loop only breaks after the pose is confirmed
pose = initialise_pose.confirm_pose(pose, sensor_readings) 
paths = []
if pose[0] < 1500:
    paths = cw_paths
else:
    paths = ccw_paths

path_idx = 1 # skip first path

odometry.reset_pose()
while True:
    sensor_readings = sensors.read()
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
    else:
        pose = odometry_pose
    
    path_changed, path_idx = nav.drive_paths(path_idx, paths, pose, SPEED)
    if path_idx >= PATHS_LIMIT:
        break

nav.stop()

    





