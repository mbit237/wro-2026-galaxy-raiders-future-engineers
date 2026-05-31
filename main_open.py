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
QUARTER_SECTIONS = 16

devices = initialise_hardware.init()
pose = initialise_pose.open(devices)

# Check if first wall is extended 
if pose[3]:
    open_first_path[0][0] = 300
    open_first_path[1][0] = 300
        
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

    if navigation.drive_path(open_first_path, pose, SPEED):
        if localised_pose:
            break

pose = initialise_pose.confirm_pose(pose, sensor_readings)
paths = []
if pose[0] < 1500:
    paths = cw_paths
else:
    paths = ccw_paths

path_idx = 1 # skip first path
path_quarter_sections = 0

odometry.reset_pose()
while True:
    sensor_readings = sensors.read()
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
    else:
        pose = odometry_pose
    
    idx = navigation.drive_paths(path_idx, paths, pose, SPEED)
    # Next quarter section
    if idx % 4 == 0: 
        idx = 0 
        path_quarter_sections += 1
    # Went past last quarter section
    if path_quarter_sections >= (QUARTER_SECTIONS - 1):
        break

navigation.stop()

    





