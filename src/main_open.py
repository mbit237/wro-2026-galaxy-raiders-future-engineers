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
from paths import open_first_path, cw_open_outer_paths, ccw_open_outer_paths, cw_open_inner_paths, ccw_open_inner_paths

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 4
SEGMENT_DIVIDER = 1700
STOPPING_TOP_POS = 1800
STOPPING_BOTTOM_POS = 1300

MODE = "obstacle"

devices = initialise_hardware.init()
nav = navigation.Navigation(devices)

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
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        # pose = complementary_filter.merge(odometry_pose, localised_pose)
        pose = odometry_pose
        print(f"odo_pose: {odometry_pose}, localised_pose: {localised_pose}")
#        print(pose)
    else:
        pose = odometry_pose

#    print(pose)
    if nav.drive_path(open_first_path, pose, SPEED):
        break

# ----- Change it such that the while loop only breaks after the pose is confirmed ----- #
print("Checking position")
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        print(pose)
        tmp_pose = initialise_pose.confirm_pose(pose, sensor_readings)
        if tmp_pose != None:
            pose = tmp_pose
            break
    pose = odometry_pose


paths = []
if pose[0] < 1500:
    paths = cw_open_outer_paths
    inner_paths = cw_open_inner_paths
    check_wall_dir = "right"
    print('cw')
else:
    paths = ccw_open_outer_paths
    inner_paths = ccw_open_inner_paths
    check_wall_dir = "left"
    print('ccw')
path_idx = 1 # skip first path

print('After first path: ', pose)


# --------------- Main Loop --------------- # 
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print('Localised pose: ', pose)
        # if abs(localised_pose[2] - odometry_pose[2]) > 9:
        #     break
    else:
        pose = odometry_pose

    path_changed, path_idx = nav.drive_paths(path_idx, paths, pose, SPEED)
    if path_changed:
        print('path_changed: ', path_idx)
        if not initialise_pose.open_check_wall_extended(devices, check_wall_dir):
            print(f'changed paths list at idx, {path_idx}')
            paths[path_idx % 4] = inner_paths[path_idx % 4]
        print('paths list: ', paths, path_idx)

    if path_idx >= PATHS_LIMIT:
        if segment == 'bottom' and pose[1] >= STOPPING_BOTTOM_POS:
            break
        elif segment == 'top' and pose[1] >= STOPPING_TOP_POS:
            break

nav.stop()

    





