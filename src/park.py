import math
import time

import drive as drive
import odometry as odometry
import sensors as sensors

def park(devices, pose, parking_path):
    # parking_path determines whether the parking area is on the left or right
    drive = devices["drive"]
    side = -1  # left side
    if parking_path[0][0] < 1500:
        side = -1
    else:
        side = 1  # right side
    start_heading = pose[2]
    first_turn_heading = (start_heading + side * 45) % 360

    print("starting parking procedure")
    print("parking pose:", pose)

    # turn towards the parking space and reverse
    phase_start = pose
    start_time = time.time()
    while time.time() - start_time < 6:
        heading_error = (first_turn_heading - pose[2] + 180) % 360 - 180
        travelled = math.sqrt((pose[0] - phase_start[0]) ** 2 + (pose[1] - phase_start[1]) ** 2)
        if abs(heading_error) < 5 and travelled > 120:
            break
        sensor_readings = sensors.read(devices)
        pose = odometry.estimate_pose(pose, sensor_readings)
        drive.steering(side * drive.STEER_MAX)
        drive.drive(-170)

    # straighten the wheels and reverse past the front vehicle
    phase_start = pose
    start_time = time.time()
    while time.time() - start_time < 6:
        travelled = math.sqrt((pose[0] - phase_start[0]) ** 2 + (pose[1] - phase_start[1]) ** 2)
        if travelled > 260:
            break
        sensor_readings = sensors.read(devices)
        pose = odometry.estimate_pose(pose, sensor_readings)
        drive.steering(0)
        drive.drive(-170)

    # turn away from the parking space and reverse until parallel
    phase_start = pose
    start_time = time.time()
    while time.time() - start_time < 6:
        heading_error = (start_heading - pose[2] + 180) % 360 - 180
        travelled = math.sqrt((pose[0] - phase_start[0]) ** 2 + (pose[1] - phase_start[1]) ** 2)
        if abs(heading_error) < 7 and travelled > 100:
            break
        sensor_readings = sensors.read(devices)
        pose = odometry.estimate_pose(pose, sensor_readings)
        drive.steering(-side * drive.STEER_MAX)
        drive.drive(-150)

    # move forward with straight wheels to centre the robot
    phase_start = pose
    start_time = time.time()
    while time.time() - start_time < 4:
        travelled = math.sqrt((pose[0] - phase_start[0]) ** 2 + (pose[1] - phase_start[1]) ** 2)
        if travelled > 140:
            break
        sensor_readings = sensors.read(devices)
        pose = odometry.estimate_pose(pose, sensor_readings)
        drive.steering(0)
        drive.drive(130)

    drive.steering(0)
    drive.drive(0)

    print("parked")
    return pose
