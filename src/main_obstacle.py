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

USE_TELEMETRY = False
SPEED = 250 
PATHS_LIMIT = 48  # full run is 48, 16 per round
MODE = "obstacle"

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

def parking_cw():
    global pose

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
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < (pose[2] % 360) < 93)):
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
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < (pose[2] % 360) < 93)):
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

    print("end pose: ", pose)

def parking_ccw():
    global pose

    print("starting parking procedure", pose)

    fwd_stop_y = parking_path[1][1]
    rear_stop_y = 1800
    x_min = parking_path[0][0] - 15
    x_max = parking_path[0][0] + 15
    y_min = 2140 - 5
    y_max = 2140 + 5

    prev_time = time.time()
    parking_start_pos_reached = False

    # Parking stage 1
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
                # print(pose)
                prev_time = time.time()
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < (pose[2] % 360) < 93)):
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
                # print(pose)
                prev_time = time.time()
            if ((x_min < pose[0] < x_max) and (y_min < pose[1] < y_max) and (87 < (pose[2] % 360) < 93)):
                parking_start_pos_reached = True
                break

    nav.stop()

    # Parking stage 2

    # Back 1: 45 deg
    devices["drive"].steering(45)
    devices["drive"].drive(-150)

    move_back_1_stop_y = 2015 
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

    # Back 2: 0 deg
    sleep_with_update(0.3)
    devices["drive"].steering(0)
    sleep_with_update(0.3)

    devices["drive"].drive(-150)

    move_back_1_stop_y = 1950 
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

    # Back 3: -45 deg
    sleep_with_update(0.3)
    devices["drive"].steering(-45)
    sleep_with_update(0.3)

    devices["drive"].drive(-100)

    move_back_2_stop_y = 1880
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

    # Forward: straigten to heading 90
    forward_stop_y = 1930
    while pose[1] < forward_stop_y:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose
        devices["drive"].steer_p(90, pose[2], 75)

    devices["drive"].drive(0)
    sleep_with_update(0.3)
    print('end pose end of stage 2: ', pose) 

    # Stage 3
    while pose[0] < 2880:
        # Back 1
        devices["drive"].steering(45)
        sleep_with_update(0.1)
        devices["drive"].drive(-75)

        move_back_2_stop_y = 1910
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

        devices["drive"].steering(-45)
        sleep_with_update(0.3)
        devices["drive"].drive(-75)

        move_back_2_stop_y = 1860
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

        sleep_with_update(0.3)
        print('end pose stage 2-2: ', pose) 

        # forward
        devices["drive"].steering(45)
        sleep_with_update(0.3)
        devices["drive"].drive(75)

        forward_stop_y = 1880
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

        forward_stop_y = 1935
        while pose[1] < forward_stop_y: 
            sensor_readings = sensors.read(devices)
            odometry_pose = odometry.estimate_pose(pose, sensor_readings)
            localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
            if localised_pose: 
                pose = complementary_filter.merge(odometry_pose, localised_pose)
            else:
                pose = odometry_pose
            devices["drive"].steer_p(90, pose[2], 75)

        devices["drive"].drive(0)
        sleep_with_update(0.3)

        print('end pose stage 2-4: ', pose) 
    
    print("end pose: ", pose)

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

park_func = ""

if pose[0] < 1500:
    red_paths = cw_obstacle_inner_paths
    green_paths = cw_obstacle_outer_paths
    inner_starting_paths = cw_obstacle_first_inner_paths
    outer_starting_paths = cw_obstacle_first_outer_paths
    parking_path = cw_parking_path
    obstacle_positions = cw_obstacle_positions
    park_func = "cw"
else:
    green_paths = ccw_obstacle_inner_paths
    red_paths = ccw_obstacle_outer_paths
    inner_starting_paths = ccw_obstacle_first_inner_paths
    outer_starting_paths = ccw_obstacle_first_outer_paths
    parking_path = ccw_parking_path
    obstacle_positions = ccw_obstacle_positions
    park_func = "ccw"

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
if pose[0] < 1500:
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
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
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        nav.drive_path_back(starting_paths[1], pose, 200, debug=False)
        if time.time() > stop_time:
            break
    print('exit 2')

    # move a bit more so that bot wont brush parking wall
    starting_paths[0][2] -= 40
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        if nav.drive_path(starting_paths[0], pose, 200, debug=False):
            break
    print('exit 3')

    stop_time = time.time() + 0.35
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        nav.drive_path_back(starting_paths[1], pose, 200, debug=False)
        if time.time() > stop_time:
            break
    print('exit 4')

    # get out of parking: forward2
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        if nav.drive_path(starting_paths[2], pose, 200, debug=False):
            break
    print('exit 5')

    color = cam.detect_blob()
    print(color)
    if color == "r":
        print("red")
        starting_paths = inner_starting_paths
        paths = red_paths
    else:
        print("green")
        starting_paths = outer_starting_paths
        paths = green_paths

elif pose[0] > 1500:
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

        if nav.drive_path(starting_paths[0], pose, 100, debug=False):
            break
    print('exit 1')

    # # get out of parking: back
    stop_time = time.time() + 0.30
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        nav.drive_path_back(starting_paths[1], pose, 100, debug=False)
        if time.time() > stop_time:
            break
    print('exit 2')

    # move a bit more so that bot wont brush parking wall
    starting_paths[0][2] -= 40
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        if nav.drive_path(starting_paths[0], pose, 100, debug=False):
            break
    print('exit 3')

    stop_time = time.time() + 0.30
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        nav.drive_path_back(starting_paths[1], pose, 100, debug=False)
        if time.time() > stop_time:
            break
    print('exit 4')

    # get out of parking: forward2
    while True:
        sensor_readings = sensors.read(devices)
        odometry_pose = odometry.estimate_pose(pose, sensor_readings)
        localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
        if localised_pose: 
            pose = complementary_filter.merge(odometry_pose, localised_pose)
            print("localised pose: ", odometry_pose, localised_pose, pose)
        else:
            pose = odometry_pose

        dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
        devices["camera_servo"].set_dir(dir_to_obstacle)

    #    print(pose)
        if nav.drive_path(starting_paths[2], pose, 200, debug=False):
            break
    print('exit 5')

    color = cam.detect_blob()
    print(color)
    if color == "r":
        print("red")
        starting_paths = outer_starting_paths
        paths = red_paths
    else:
        print("green")
        starting_paths = inner_starting_paths
        paths = green_paths

print('obs', pose, obstacle_positions[-1])



# get out of parking: forward3 (run a bit more if neccessary)
while True:
    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    pose = odometry_pose

    dir_to_obstacle = dir_to_point(pose, obstacle_positions[-1])
    devices["camera_servo"].set_dir(dir_to_obstacle)

#    print(pose)
    if nav.drive_path(starting_paths[2], pose, 200, debug=False):
        break
print('exit 6')

while True:
    # debugging prints
    now = time.time()
    if now >= next_print:
        debug = True
    else:
        debug = False


    sensor_readings = sensors.read(devices)
    odometry_pose = odometry.estimate_pose(pose, sensor_readings)
    localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
    if localised_pose: 
        pose = complementary_filter.merge(odometry_pose, localised_pose)
        print("localised pose: ", odometry_pose, localised_pose, pose)
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

print(pose)

# --- Parking Procedure --- #

if park_func == "cw":
    parking_cw()
else:
    parking_ccw()

nav.stop()

# 1. turn steering wheels to the max left
# 2. drive backward 20/2  = 10 cm
# 3. turn steering wheels to the max right
# 4. drive backward 20/2 = 10 cm
# 5. turn steering wheels to the max left
# 6. drive forward and back forward and back till fully inside 


# time.sleep(2)
# Post run debugging
# localisation_count = 0
# while True:
#     sensor_readings = sensors.read(devices)
#     odometry_pose = odometry.estimate_pose(pose, sensor_readings)
#     localised_pose = localisation.localise(odometry_pose, sensor_readings, MODE)
#     if localised_pose: 
#         localisation_count += 1
#         pose = complementary_filter.merge(odometry_pose, localised_pose)
#         print("localised pose: ", odometry_pose, localised_pose, pose)
#     else:
#         pose = odometry_pose

#     if localisation_count == 10:
#         break
