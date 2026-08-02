import time

import initialise_hardware
import initialise_pose
import telemetry_client
import sensors
import odometry
import spike_localisation as localisation
import complementary_filter
import navigation
import rpicam
import led
from utilities import * 
from paths import cw_obstacle_inner_paths, cw_obstacle_outer_paths, ccw_obstacle_inner_paths, ccw_obstacle_outer_paths, cw_parking_path, ccw_parking_path
from obstacles import cw_obstacle_positions

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 3

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

pose = initialise_pose.obstacle_on_path(devices)
print('initial pose =', pose)

devices['led'].green_on()

if pose[0] < 1500:
    red_paths = cw_obstacle_inner_paths
    green_paths = cw_obstacle_outer_paths
    parking_path = cw_parking_path
else:
    green_paths = ccw_obstacle_inner_paths
    red_paths = ccw_obstacle_outer_paths
    parking_path = ccw_parking_path

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
paths = red_paths

obstacle_position = cw_obstacle_positions[path_idx]
print("obstalce_position: ", obstacle_position)
dir_to_obstacle = dir_to_point(pose, obstacle_position)
print("dir_to_obstacle: ", dir_to_obstacle)
devices["camera_servo"].set_dir(dir_to_obstacle)
time.sleep(1)

color = cam.detect_blob()
print(color)
if color == "r":
    paths = red_paths
elif color == "g":
    paths = green_paths

# Main Loop
odometry.reset_pose()
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
    else:
        pose = odometry_pose
    
    path_changed, path_idx = nav.drive_paths(path_idx, paths, pose, SPEED)
    if path_idx >= PATHS_LIMIT:
        break
    
    # Aim servo at obstacles
    obstacle_position = cw_obstacle_positions[path_idx]
    dir_to_obstacle = dir_to_point(pose, obstacle_position)
    devices["camera_servo"].set_dir(dir_to_obstacle)
    color = cam.detect_blob() # always have updated image
    # infinite impulse response filter (running average) or finite impulse response filter (recent average)
    
    if path_changed:
        if color == "r":
            paths = red_paths
        elif color == "g":
            paths = green_paths

nav.stop()

    





