import time

import drive
import initialise_hardware as initialise_hardware
import initialise_pose as initialise_pose
import telemetry_client as telemetry_client
import sensors as sensors
import odometry as odometry
import point_cloud_localisation as localisation
import complementary_filter as complementary_filter
import navigation as navigation
import rpicam as rpicam
import led as led
from utilities import * 
from paths import cw_obstacle_inner_paths, cw_obstacle_outer_paths, ccw_obstacle_inner_paths, ccw_obstacle_outer_paths, cw_parking_path, ccw_parking_path, cw_obstacle_first_inner_paths, cw_obstacle_first_outer_paths, ccw_obstacle_first_inner_paths, ccw_obstacle_first_outer_paths
from obstacles import cw_obstacle_positions, ccw_obstacle_positions


def sleep_with_update(dur, debug=False):
    global pose

    end_time = time.time() + dur
    loc = 0
    while time.time() < end_time:
        sensor_readings = sensors.read(devices)
        localised_pose = localisation.localise(pose, sensor_readings, MODE)
        if localised_pose: 
            # print(sensor_readings['lidar'])
            loc += 1
            pose = localised_pose
            if debug:
                print('sleep', localised_pose)

    if debug:
        print(pose, loc)

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 17  # full run is 37
MODE = "obstacle"

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
pose = [730, 1040, 80]
# pose = [2680, 2150, 90]

sleep_with_update(0.5)

devices['led'].green_on()

parking_path = cw_parking_path

# [[375, 1350], [375, 2000]]

# --- Parking Procedure --- #

print("starting parking procedure", pose)

fwd_stop_y = parking_path[1][1]
rear_stop_y = 1080
x_min = parking_path[0][0] - 15
x_max = parking_path[0][0] + 15
y_min = 1410 - 5
y_max = 1410 + 5

prev_time = time.time()
parking_start_pos_reached = False

while True:

    # Move Forward
    while pose[1] < fwd_stop_y: 
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose
        nav.drive_path(parking_path, pose, 175)
        if (time.time() - prev_time) > 0.5:
            print(pose)
            prev_time = time.time()
        if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
            parking_start_pos_reached = True
            break

    if parking_start_pos_reached:
        devices["drive"].drive(0)
        devices["drive"].steering(0)
        print("parking starting pos pt 1 reached", pose)
        break    

    # Move backward
    while pose[1] > rear_stop_y: 
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        nav.drive_path_back(parking_path, pose, 200)
        if (time.time() - prev_time) > 0.5:
            print(pose)
            prev_time = time.time()
        if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < pose[2] < 93)):
            parking_start_pos_reached = True
            break

nav.stop()

# Parking 
devices["drive"].steering(-45)
devices["drive"].drive(-150)

move_back_1_stop_y = 1290
while pose[1] > move_back_1_stop_y: 
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print("localised pose: ", odometry_pose, localised_pose, pose)
    else:
        pose = odometry_pose

devices["drive"].drive(0)
print('end pose1: ', pose) 

sleep_with_update(0.3)
devices["drive"].steering(0)
sleep_with_update(0.3)

devices["drive"].drive(-150)

move_back_1_stop_y = 1230
while pose[1] > move_back_1_stop_y: 
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print("localised pose: ", odometry_pose, localised_pose, pose)
    else:
        pose = odometry_pose


print('end pose3: ', pose) 
devices["drive"].drive(0)

sleep_with_update(0.3)
devices["drive"].steering(45)
sleep_with_update(0.3)

devices["drive"].drive(-100)

move_back_2_stop_y = 1160
while pose[1] > move_back_2_stop_y: 
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
    else:
        pose = odometry_pose

print('end pose4: ', pose) 
devices["drive"].drive(0)
# devices["drive"].steering(45)
sleep_with_update(0.3)

forward_stop_y = 1200
while pose[1] < forward_stop_y:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print("localised pose: ", odometry_pose, localised_pose, pose)
    else:
        pose = odometry_pose
    devices["drive"].steer_p(90, pose[2], 75, gain=4)

devices["drive"].drive(0)
sleep_with_update(0.3)
print('end pose end of stage 2: ', pose) 

# Stage 3
while pose[0] > 120:
    # Back 1
    devices["drive"].steering(-45)
    sleep_with_update(0.1)
    devices["drive"].drive(-75)

    move_back_2_stop_y = 1180
    while pose[1] > move_back_2_stop_y: 
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
        else:
            pose = odometry_pose

    devices["drive"].drive(0)
    sleep_with_update(0.3)
    print("end pose stage 2-1: ", pose)

    devices["drive"].steering(45)
    sleep_with_update(0.3)
    devices["drive"].drive(-75)

    move_back_2_stop_y = 1130
    while pose[1] > move_back_2_stop_y: 
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
        else:
            pose = odometry_pose

    devices["drive"].drive(0)
    sleep_with_update(0.3)

    nav.stop()
    sleep_with_update(0.3)
    print('end pose stage 2-2: ', pose) 

    # forward
    devices["drive"].steering(-45)
    sleep_with_update(0.3)
    devices["drive"].drive(75)

    forward_stop_y = 1150
    while pose[1] < forward_stop_y: 
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
        else:
            pose = odometry_pose

    devices["drive"].drive(0)
    sleep_with_update(0.3)
    print("end pose stage 2-3: ", pose)

    forward_stop_y = 1205
    while pose[1] < forward_stop_y: 
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
        else:
            pose = odometry_pose
        devices["drive"].steer_p(90, pose[2], 75, gain=4)

    devices["drive"].drive(0)
    sleep_with_update(0.3)

    print('end pose stage 2-4: ', pose) 

nav.stop()
print("end pose: ", pose)