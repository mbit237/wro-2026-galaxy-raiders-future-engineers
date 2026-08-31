import time

import initialise_hardware as initialise_hardware
import initialise_pose as initialise_pose
import telemetry_client as telemetry_client
import sensors as sensors
import odometry as odometry
import spike_localisation as localisation
import complementary_filter as complementary_filter
import navigation as navigation
import rpicam as rpicam
import led as led
from utilities import * 
from paths import cw_obstacle_inner_paths, cw_obstacle_outer_paths, ccw_obstacle_inner_paths, ccw_obstacle_outer_paths, cw_parking_path, ccw_parking_path, cw_obstacle_first_inner_paths, cw_obstacle_first_outer_paths, ccw_obstacle_first_inner_paths, ccw_obstacle_first_outer_paths
from obstacles import cw_obstacle_positions, ccw_obstacle_positions

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 3  # full run is 37

devices = initialise_hardware.init()
nav = navigation.Navigation(devices)
cam = rpicam.Rpicam()
        
if USE_TELEMETRY:
    telemetry_client.connect()

devices['led'].all_off()
time.sleep(1)
devices['led'].yellow_on()
print("wait for button")
# display LED colour to show it is ready and the mode (obstacle or open)
while True:
    if devices["pi"].read(17) == 0:
        time.sleep(0.5)
        break 

devices['led'].yellow_off()

devices['lidar'].flush()
# pose = initialise_pose.obstacle_on_path(devices)
pose = initialise_pose.obstacle_in_parking(devices)
print('initial pose =', pose)

devices['led'].green_on()

if pose[0] < 1500:
    red_paths = cw_obstacle_inner_paths
    green_paths = cw_obstacle_outer_paths
    inner_starting_paths = cw_obstacle_first_inner_paths
    outer_starting_paths = cw_obstacle_first_outer_paths
    parking_path = cw_parking_path
    obstacle_positions = cw_obstacle_positions
else:
    green_paths = ccw_obstacle_inner_paths
    red_paths = ccw_obstacle_outer_paths
    inner_starting_paths = ccw_obstacle_first_inner_paths
    outer_starting_paths = ccw_obstacle_first_outer_paths
    parking_path = ccw_parking_path
    obstacle_positions = ccw_obstacle_positions

# Reverse if necessary 
# L_dist = min(get_distance(ldr, 20), get_distance(ldr, 25), get_distance(ldr, 30), get_distance(ldr, 35), get_distance(ldr, 40), get_distance(ldr, 45))
# R_dist = min(get_distance(ldr, 340), get_distance(ldr, 335), get_distance(ldr, 330), get_distance(ldr, 325), get_distance(ldr, 320), get_distance(ldr, 315))
# reverse = False
# print("L_dist: ",  L_dist)
# if pose[0] < 1500: #left
#     if R_dist < 400:
#         colour = rpicam.detect_blob()
#         if colour == "r":
#             reverse = True
#             print(colour)
#             paths = obstacle_inner_paths
#         else:
#             print(colour)
#             paths = obstacle_outer_paths
# else:
#     if L_dist < 400:
#         colour = rpicam.detect_blob()
#         if colour == "r":
#             reverse = True
#             print(colour)
#             paths = obstacle_outer_paths
#         else:
#             print(colour)
#             paths = obstacle_inner_paths

path_idx = 0 
# obstacle_position = obstacle_positions[path_idx]
# obstacle_position = obstacle_positions[-1]

# print("obstacle_position: ", obstacle_position)
# dir_to_obstacle = dir_to_point(pose, obstacle_position)
# print("dir_to_obstacle: ", dir_to_obstacle)
#devices["camera_servo"].set_dir(dir_to_obstacle)
#time.sleep(1)

# Main Loop
odometry.reset_pose()
print_period = 0.5
next_print = time.time() + print_period

# get out of parking: forward
starting_paths = outer_starting_paths
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    pose = odometry_pose

    dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
    devices["camera_servo"].set_dir(dir_to_obstacle)

    if nav.drive_path(starting_paths[0], pose, 200, debug=False):
        break
print('exit 1')

# # get out of parking: back
stop_time = time.time() + 0.35
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    pose = odometry_pose

    dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
    devices["camera_servo"].set_dir(dir_to_obstacle)

#    print(pose)
    nav.drive_path_back(starting_paths[1], pose, 200, debug=False)
    if time.time() > stop_time:
        break
print('exit 2')

# # get out of parking: forward2
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    pose = odometry_pose

    dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
    devices["camera_servo"].set_dir(dir_to_obstacle)

#    print(pose)
    if nav.drive_path(starting_paths[2], pose, 200, debug=False):
        break
print('exit 3')

print('obs', pose, obstacle_positions[-1])

color = cam.detect_blob()
print(color)
if color == "r":
    print("red")
    starting_paths = inner_starting_paths
    paths = red_paths
elif color == "g":
# else:
    print("green")
    starting_paths = outer_starting_paths
    paths = green_paths

# # get out of parking: forward3 (run a bit more if neccessary)
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    pose = odometry_pose

    dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
    devices["camera_servo"].set_dir(dir_to_obstacle)

#    print(pose)
    if nav.drive_path(starting_paths[2], pose, 200, debug=False):
        break
print('exit 4')

while True:
    # debugging prints
    now = time.time()
    if now >= next_print:
        debug = True
    else:
        debug = False


    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print("localised pose: ", pose)
    else:
        pose = odometry_pose
    
    path_changed, path_idx = nav.drive_paths(path_idx, paths, pose, SPEED, debug=debug)
    if path_idx >= PATHS_LIMIT:
        break
    
    # Aim servo at obstacles
    obstacle_position = obstacle_positions[(path_idx) % len(obstacle_positions)]
    dir_to_obstacle = dir_to_point(pose, obstacle_position)
    devices["camera_servo"].set_dir(dir_to_obstacle)
    color = cam.detect_blob() # always have updated image
    # infinite impulse response filter (running average) or finite impulse response filter (recent average)
    
    if debug:
        next_print += print_period
        print('pose', pose)
        print('dir to obs', dir_to_obstacle, obstacle_position)
        print('color', color)

    if path_changed:
        if color == "r":
            print('path changed r', path_idx)
            paths = red_paths
            print('new path', paths[path_idx % len(paths)])
        elif color == "g":
            print('path changed g', path_idx)
            paths = green_paths
            print('new path', paths[path_idx % len(paths)])
        else:
            print('path changed n', path_idx)

nav.stop()
