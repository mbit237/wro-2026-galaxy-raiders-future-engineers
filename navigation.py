import math
import drive

PATH_GAIN = -0.2
MAX_ANGLE = 30

def dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def drive_path(path, pose, speed):
    target_dir = path[3]
    robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
    
    err = dot(path[5], robot_vec) # how far off the robot is (in mm)
    corr = err * PATH_GAIN 
    # Limit correction
    if corr > MAX_ANGLE:
        corr = MAX_ANGLE
    elif corr < -MAX_ANGLE:
        corr = -MAX_ANGLE
    
    target_dir += corr 
    # print(f"Target direction: {target_dir}, gyro: {pose}")
    drive.steer_p(target_dir, pose[2], speed)
    robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
    dist_travelled_along_path = dot(path[4], robot_vec)
    
    if dist_travelled_along_path >= path[2]:
        return True
    else:
        return False

def drive_path_back(path, pose, speed):
    target_dir = path[3] 
    robot_vec = [pose[0] - path[0][0], pose[1] - path[0][1]]
    
    err = dot(path[5], robot_vec) # how far off the robot is (in mm)
    # corr = err * PATH_GAIN
    corr = 0 
    # Limit correction
    if corr > MAX_ANGLE:
        corr = MAX_ANGLE
    elif corr < -MAX_ANGLE:
        corr = -MAX_ANGLE

    target_dir -= corr 
    # print(f"Target direction: {target_dir}, gyro: {pose}")
    drive.steer_p_back(target_dir, pose[2], speed)

def drive_paths(idx, paths, pose, speed): # idx -- references path currently following 
    path = paths[idx]
    if drive_path(path, pose, speed):
        idx += 1

    return idx

def stop():
    drive.drive(0)
    drive.steering(0)